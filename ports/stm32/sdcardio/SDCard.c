/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jeff Epler for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// This implementation largely follows the structure of adafruit_sdcard.py

//#include "shared-bindings/busio/SPI.h"
//#include "shared-bindings/digitalio/DigitalInOut.h"
//#include "shared-bindings/time/__init__.h"
//#include "shared-bindings/util.h"
//#include "shared-module/sdcardio/SDCard.h"
#include "SDCard.h"
#include "pin.h"
#include "py/mperrno.h"

#if 0
#define DEBUG_PRINT(...) ((void)mp_printf(&mp_plat_print, ##__VA_ARGS__))
#else
#define DEBUG_PRINT(...) ((void)0)
#endif

#define CMD_TIMEOUT (300)

#define R1_IDLE_STATE (1 << 0)
#define R1_ILLEGAL_COMMAND (1 << 2)

#define TOKEN_CMD25 (0xFC)
#define TOKEN_STOP_TRAN (0xFD)
#define TOKEN_DATA (0xFE)

#define NO_SD_CARD -1
#define COULDNT_DETERMINE_SD_CARD_VERSION -2
#define NO_RESPONSE_FROM_SD_CARD -3
#define SD_CARD_CSD_FORMAT_NOT_SUPPORTED -4
#define CANT_SET_512_BLOCK_SIZE -5
#define TIMEOUT_WAITING_FOR_V1_CARD -6
#define TIMEOUT_WAITING_FOR_V2_CARD -7
#define BUFFER_LENGTH_MUST_BE_A_MULTIPLE_OF_512 -8

bool spi_write(sdcardio_sdcard_obj_t *self, const uint8_t *data, size_t len)
{
    spi_transfer(self->bus, len, data, NULL, SPI_TRANSFER_TIMEOUT(len));

    return true;
}

bool spi_read(sdcardio_sdcard_obj_t *self, uint8_t *data, size_t len, uint8_t write_value)
{
    spi_transfer(self->bus, len, &write_value, data, SPI_TRANSFER_TIMEOUT(len));
    return true;
}

// STATIC bool lock_and_configure_bus(sdcardio_sdcard_obj_t *self) {
//    if (!common_hal_busio_spi_try_lock(self->bus)) {
//        return false;
//    }
//    common_hal_busio_spi_configure(self->bus, self->baudrate, 0, 0, 8);
//    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
//    return true;
// }

// STATIC void lock_bus_or_throw(sdcardio_sdcard_obj_t *self) {
//    if (!lock_and_configure_bus(self)) {
//        mp_raise_OSError(EAGAIN);
//    }
// }

STATIC void clock_card(sdcardio_sdcard_obj_t *self, int bytes)
{
    uint8_t buf[] = {0xff};
    //    common_hal_digitalio_digitalinout_set_value(&self->cs, true);
    mp_hal_pin_high(self->cs);
    for (int i = 0; i < bytes; i++)
    {
        //    common_hal_busio_spi_write(self->bus, buf, 1);
        spi_write(self, buf, 1);
    }
}

//STATIC void extraclock_and_unlock_bus(sdcardio_sdcard_obj_t *self) {
//    clock_card(self, 1);
//    common_hal_busio_spi_unlock(self->bus);
//}
//
static uint8_t CRC7(const uint8_t *data, uint8_t n)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t d = data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            crc <<= 1;
            if ((d & 0x80) ^ (crc & 0x80))
            {
                crc ^= 0x09;
            }
            d <<= 1;
        }
    }
    return (crc << 1) | 1;
}

#define READY_TIMEOUT_US (300 * 1000) // 300ms
STATIC void wait_for_ready(sdcardio_sdcard_obj_t *self)
{
	mp_uint_t deadline = mp_hal_ticks_us() + READY_TIMEOUT_US;
    while (mp_hal_ticks_us() < deadline)
    {
        uint8_t b;
        //    common_hal_busio_spi_read(self->bus, &b, 1, 0xff);
        spi_read(self, &b, 1, 0xff);
        if (b == 0xff)
        {
            break;
        }
    }
}
//// In Python API, defaults are response=None, data_block=True, wait=True
STATIC int cmd(sdcardio_sdcard_obj_t *self, int cmd, int arg, void *response_buf, size_t response_len, bool data_block, bool wait)
{
    DEBUG_PRINT("cmd % 3d [%02x] arg=% 11d [%08x] len=%d%s%s\n", cmd, cmd, arg, arg, response_len, data_block ? " data" : "", wait ? " wait" : "");
    uint8_t cmdbuf[6];
    cmdbuf[0] = cmd | 0x40;
    cmdbuf[1] = (arg >> 24) & 0xff;
    cmdbuf[2] = (arg >> 16) & 0xff;
    cmdbuf[3] = (arg >> 8) & 0xff;
    cmdbuf[4] = arg & 0xff;
    cmdbuf[5] = CRC7(cmdbuf, 5);

    if (wait)
    {
        wait_for_ready(self);
    }
    //    common_hal_busio_spi_write(self->bus, cmdbuf, sizeof(cmdbuf));
    spi_write(self, cmdbuf, sizeof(cmdbuf));

    // Wait for the response (response[7] == 0)
    bool response_received = false;
    for (int i = 0; i < CMD_TIMEOUT; i++)
    {
        //    common_hal_busio_spi_read(self->bus, cmdbuf, 1, 0xff);
        spi_read(self, cmdbuf, 1, 0xff);
        if ((cmdbuf[0] & 0x80) == 0)
        {
            response_received = true;
            break;
        }
    }

    if (!response_received)
    {
        return -EIO;
    }

    if (response_buf)
    {

        if (data_block)
        {
            cmdbuf[1] = 0xff;
            do
            {
                // Wait for the start block byte
                //common_hal_busio_spi_read(self->bus, cmdbuf + 1, 1, 0xff);
                spi_read(self, cmdbuf + 1, 1, 0xff);
            } while (cmdbuf[1] != 0xfe);
        }

        //    common_hal_busio_spi_read(self->bus, response_buf, response_len, 0xff);
        spi_read(self, response_buf, response_len, 0xff);

        if (data_block)
        {
            // Read and discard the CRC-CCITT checksum
            //    common_hal_busio_spi_read(self->bus, cmdbuf + 1, 2, 0xff);
            spi_read(self, cmdbuf + 1, 2, 0xff);
        }
    }

    return cmdbuf[0];
}

//// In Python API, defaults are response=None, data_block=True, wait=True
STATIC int cmd_crc(sdcardio_sdcard_obj_t *self, int cmd, int arg, void *response_buf, size_t response_len, bool data_block, bool wait, bool crc)
{
    DEBUG_PRINT("cmd % 3d [%02x] arg=% 11d [%08x] len=%d%s%s\n", cmd, cmd, arg, arg, response_len, data_block ? " data" : "", wait ? " wait" : "");
    uint8_t cmdbuf[6];
    cmdbuf[0] = cmd | 0x40;
    cmdbuf[1] = (arg >> 24) & 0xff;
    cmdbuf[2] = (arg >> 16) & 0xff;
    cmdbuf[3] = (arg >> 8) & 0xff;
    cmdbuf[4] = arg & 0xff;
    if(crc){
    	cmdbuf[5] = CRC7(cmdbuf, 5);
    }
    else{
    	cmdbuf[5] = 0;
    }

    if (wait)
    {
        wait_for_ready(self);
    }
    //    common_hal_busio_spi_write(self->bus, cmdbuf, sizeof(cmdbuf));
    spi_write(self, cmdbuf, sizeof(cmdbuf));

    // Wait for the response (response[7] == 0)
    bool response_received = false;
    for (int i = 0; i < CMD_TIMEOUT; i++)
    {
        //    common_hal_busio_spi_read(self->bus, cmdbuf, 1, 0xff);
        spi_read(self, cmdbuf, 1, 0xff);
        if ((cmdbuf[0] & 0x80) == 0)
        {
            response_received = true;
            break;
        }
    }

    if (!response_received)
    {
        return -EIO;
    }

    if (response_buf)
    {

        if (data_block)
        {
            cmdbuf[1] = 0xff;
            do
            {
                // Wait for the start block byte
                //common_hal_busio_spi_read(self->bus, cmdbuf + 1, 1, 0xff);
                spi_read(self, cmdbuf + 1, 1, 0xff);
            } while (cmdbuf[1] != 0xfe);
        }

        //    common_hal_busio_spi_read(self->bus, response_buf, response_len, 0xff);
        spi_read(self, response_buf, response_len, 0xff);

        if (data_block)
        {
            // Read and discard the CRC-CCITT checksum
            //    common_hal_busio_spi_read(self->bus, cmdbuf + 1, 2, 0xff);
            spi_read(self, cmdbuf + 1, 2, 0xff);
        }
    }

    return cmdbuf[0];
}

STATIC int block_cmd(sdcardio_sdcard_obj_t *self, int cmd_, int block, void *response_buf, size_t response_len, bool data_block, bool wait)
{
    return cmd(self, cmd_, block * self->cdv, response_buf, response_len, true, true);
}

// STATIC bool cmd_nodata(sdcardio_sdcard_obj_t *self, int cmd, int response)
// {
//     uint8_t cmdbuf[2] = {cmd, 0xff};

//     // common_hal_busio_spi_write(self->bus, cmdbuf, sizeof(cmdbuf));
//     spi_write(self, cmdbuf, sizeof(cmdbuf));
//     // Wait for the response (response[7] == response)
//     for (int i = 0; i < CMD_TIMEOUT; i++)
//     {
//         // common_hal_busio_spi_read(self->bus, cmdbuf, 1, 0xff);
//         spi_read(self, cmdbuf, 1, 0xff);
//         if (cmdbuf[0] == response)
//         {
//             return 0;
//         }
//     }
//     return -EIO;
// }

 STATIC const int init_card_v1(sdcardio_sdcard_obj_t *self)
 {
     for (int i = 0; i < CMD_TIMEOUT; i++)
     {
         if (cmd(self, 41, 0, NULL, 0, true, true) == 0)
         {
             return 0;
         }
     }
     return TIMEOUT_WAITING_FOR_V1_CARD;
 }

 STATIC const int init_card_v2(sdcardio_sdcard_obj_t *self) {
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        uint8_t ocr[4];
        mp_hal_delay_ms(50);
        cmd(self, 58, 0, ocr, sizeof(ocr), false, true);
        cmd(self, 55, 0, NULL, 0, true, true);
        if (cmd(self, 41, 0x40000000, NULL, 0, true, true) == 0) {
            cmd(self, 58, 0, ocr, sizeof(ocr), false, true);
            if ((ocr[0] & 0x40) != 0) {
                self->cdv = 1;
            }
            return 0;
        }
    }
    return TIMEOUT_WAITING_FOR_V2_CARD;
 }

STATIC const int init_card(sdcardio_sdcard_obj_t *self)
{
    clock_card(self, 10);

     //   common_hal_digitalio_digitalinout_set_value(&self->cs, false);
     mp_hal_pin_low(self->cs);
    //  CMD0: init card: should return _R1_IDLE_STATE (allow 5 attempts)
    {
        bool reached_idle_state = false;
        for (int i = 0; i < 5; i++)
//        for (int i = 0; i < 1; i++)
        {
            if (cmd(self, 0, 0, NULL, 0, true, true) == R1_IDLE_STATE)
            {
                reached_idle_state = true;
                break;
            }
        }
        if (!reached_idle_state)
        {
            //    return translate("no SD card");
        	mp_printf(&mp_plat_print, "no SD card\n");
            return NO_SD_CARD;
        }
    }

    // CMD8: determine card version
    {
        uint8_t rb7[4];
        int response = cmd(self, 8, 0x1AA, rb7, sizeof(rb7), false, true);
        if (response == R1_IDLE_STATE)
        {
            const int result = init_card_v2(self);
            if (result != 0)
            {
                return result;
            }
        }
        else if (response == (R1_IDLE_STATE | R1_ILLEGAL_COMMAND))
        {
            const int result = init_card_v1(self);
            if (result != 0)
            {
                return result;
            }
        }
        else
        {
            // return translate("couldn't determine SD card version");
        	mp_printf(&mp_plat_print, "couldn't determine card version\n");
            return COULDNT_DETERMINE_SD_CARD_VERSION;
        }
    }

    // CMD9: get number of sectors
    {
        uint8_t csd[16];
        int response = cmd(self, 9, 0, csd, sizeof(csd), true, true);
        if (response != 0)
        {
            //    return translate("no response from SD card");
            return NO_RESPONSE_FROM_SD_CARD;
            mp_printf(&mp_plat_print, "no response from SD card\n");
        }
        int csd_version = (csd[0] & 0xC0) >> 6;
        if (csd_version >= 2)
        {
            // return translate("SD card CSD format not supported");
            return SD_CARD_CSD_FORMAT_NOT_SUPPORTED;
            mp_printf(&mp_plat_print, "sd card csd format not supported\n");
        }

        if (csd_version == 1)
        {
            self->sectors = ((csd[8] << 8 | csd[9]) + 1) * 1024;
        }
        else
        {
            uint32_t block_length = 1 << (csd[5] & 0xF);
            uint32_t c_size = ((csd[6] & 0x3) << 10) | (csd[7] << 2) | ((csd[8] & 0xC) >> 6);
            uint32_t mult = 1 << (((csd[9] & 0x3) << 1 | (csd[10] & 0x80) >> 7) + 2);
            self->sectors = block_length / 512 * mult * (c_size + 1);
        }
    }

//     CMD16: set block length to 512 bytes
    {
    	clock_card(self, 3);
        mp_hal_pin_low(self->cs);

        int response = cmd_crc(self, 16, 512, NULL, 0, true, true, false);
        if (response != 0)
        {
            // return translate("can't set 512 block size");
            return CANT_SET_512_BLOCK_SIZE;
        }
    }

    return 0;
}

void common_hal_sdcardio_sdcard_construct(sdcardio_sdcard_obj_t *self, const spi_t *bus, pin_obj_t *cs, int baudrate)
{
    //   common_hal_digitalio_digitalinout_construct(&self->cs, cs);
    //   common_hal_digitalio_digitalinout_switch_to_output(&self->cs, true, DRIVE_MODE_PUSH_PULL);
    self->bus = bus;
    self->cs = cs;
    self->cdv = 512;
    self->sectors = 0;
//    self->baudrate = 250000;
    /*!< SPI configuration */
    SPI_InitTypeDef *init = &self->bus->spi->Init;
    init->Mode = SPI_MODE_MASTER;
    init->Direction = SPI_DIRECTION_2LINES;
    init->DataSize = SPI_DATASIZE_8BIT;
    init->CLKPolarity = SPI_POLARITY_LOW; // clock is low when idle
    init->CLKPhase = SPI_PHASE_1EDGE;     // data latched on first edge, which is rising edge for low-idle
    init->NSS = SPI_NSS_SOFT;
    init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; //SPI_BAUDRATEPRESCALER_2; // clock freq = f_PCLK / this_prescale_value; Wiz820i can do up to 80MHz
    init->FirstBit = SPI_FIRSTBIT_MSB;
    init->TIMode = SPI_TIMODE_DISABLED;
    init->CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    init->CRCPolynomial = 7; // unused
    spi_init(self->bus, false);

    //    lock_bus_or_throw(self);
    mp_hal_pin_low(self->cs);
    //    const compressed_string_t *result = init_card(self);
   const int result = init_card(self);
    //    extraclock_and_unlock_bus(self);

    if (result != 0)
    {
        //    common_hal_digitalio_digitalinout_deinit(&self->cs);
        self->cs = NULL;
        mp_raise_msg(&mp_type_OSError, NULL);
    }

    self->baudrate = baudrate;
}
//
//void common_hal_sdcardio_sdcard_deinit(sdcardio_sdcard_obj_t *self) {
//    if (!self->bus) {
//        return;
//    }
//    self->bus = 0;
//    common_hal_digitalio_digitalinout_deinit(&self->cs);
//}

void common_hal_sdcardio_check_for_deinit(sdcardio_sdcard_obj_t *self)
{
    if (!self->bus)
    {
        mp_raise_msg(&mp_type_OSError, NULL);
        //    raise_deinited_error();
    }
}

int common_hal_sdcardio_sdcard_get_blockcount(sdcardio_sdcard_obj_t *self)
{
    common_hal_sdcardio_check_for_deinit(self);
    return self->sectors;
}

int readinto(sdcardio_sdcard_obj_t *self, void *buf, size_t size)
{
    uint8_t aux[2] = {0, 0};
    while (aux[0] != 0xfe)
    {
        //    common_hal_busio_spi_read(self->bus, aux, 1, 0xff);
        spi_read(self, aux, 1, 0xff);
    }

    // common_hal_busio_spi_read(self->bus, buf, size, 0xff);
    spi_read(self, buf, size, 0xff);

    // Read checksum and throw it away
    // common_hal_busio_spi_read(self->bus, aux, sizeof(aux), 0xff);
    spi_read(self, aux, sizeof(aux), 0xff);
    return 0;
}

int readblocks(sdcardio_sdcard_obj_t *self, uint32_t start_block, mp_buffer_info_t *buf)
{
    uint32_t nblocks = buf->len / 512;
    if (nblocks == 1)
    {
        //  Use CMD17 to read a single block
        return block_cmd(self, 17, start_block, buf->buf, buf->len, true, true);
    }
    else
    {
        //  Use CMD18 to read multiple blocks
        int r = block_cmd(self, 18, start_block, NULL, 0, true, true);
        if (r < 0)
        {
            return r;
        }

        uint8_t *ptr = buf->buf;
        while (nblocks--)
        {
            r = readinto(self, ptr, 512);
            if (r < 0)
            {
                return r;
            }
            ptr += 512;
        }

        // End the multi-block read
        r = cmd(self, 12, 0, NULL, 0, true, false);

        // Return first status 0 or last before card ready (0xff)
        while (r != 0)
        {
            uint8_t single_byte;
            //    common_hal_busio_spi_read(self->bus, &single_byte, 1, 0xff);
            spi_read(self, &single_byte, 1, 0xff);
            if (single_byte & 0x80)
            {
                return r;
            }
            r = single_byte;
        }
    }
    return 0;
}

int common_hal_sdcardio_sdcard_readblocks(sdcardio_sdcard_obj_t *self, uint32_t start_block, mp_buffer_info_t *buf) {
    common_hal_sdcardio_check_for_deinit(self);
    if (buf->len % 512 != 0) {
    	mp_printf(&mp_plat_print, "Buffer length must be a multiple of 512\n");
    	mp_raise_msg(&mp_type_OSError, NULL);
    }

//    lock_and_configure_bus(self);
    mp_hal_pin_low(self->cs);
    int r = readblocks(self, start_block, buf);
    mp_hal_pin_high(self->cs);
    clock_card(self, 1);
//    extraclock_and_unlock_bus(self);
    return r;
}

//STATIC int _write(sdcardio_sdcard_obj_t *self, uint8_t token, void *buf, size_t size) {
//    wait_for_ready(self);
//
//    uint8_t cmd[2];
//    cmd[0] = token;
//
//    common_hal_busio_spi_write(self->bus, cmd, 1);
//    common_hal_busio_spi_write(self->bus, buf, size);
//
//    cmd[0] = cmd[1] = 0xff;
//    common_hal_busio_spi_write(self->bus, cmd, 2);
//
//    // Check the response
//    // This differs from the traditional adafruit_sdcard handling,
//    // but adafruit_sdcard also ignored the return value of SDCard._write(!)
//    // so nobody noticed
//    //
//    //
//    // Response is as follows:
//    //  x x x 0 STAT 1
//    //  7 6 5 4 3..1 0
//    // with STATUS 010 indicating "data accepted", and other status bit
//    // combinations indicating failure.
//    // In practice, I was seeing cmd[0] as 0xe5, indicating success
//    for (int i = 0; i < CMD_TIMEOUT; i++) {
//        common_hal_busio_spi_read(self->bus, cmd, 1, 0xff);
//        DEBUG_PRINT("i=%02d cmd[0] = 0x%02x\n", i, cmd[0]);
//        if ((cmd[0] & 0b00010001) == 0b00000001) {
//            if ((cmd[0] & 0x1f) != 0x5) {
//                return -EIO;
//            } else {
//                break;
//            }
//        }
//    }
//
//    // Wait for the write to finish
//    do {
//        common_hal_busio_spi_read(self->bus, cmd, 1, 0xff);
//    } while (cmd[0] == 0);
//
//    // Success
//    return 0;
//}
//
//STATIC int writeblocks(sdcardio_sdcard_obj_t *self, uint32_t start_block, mp_buffer_info_t *buf) {
//    common_hal_sdcardio_check_for_deinit(self);
//    uint32_t nblocks = buf->len / 512;
//    if (nblocks == 1) {
//        //  Use CMD24 to write a single block
//        int r = block_cmd(self, 24, start_block, NULL, 0, true, true);
//        if (r < 0) {
//            return r;
//        }
//        r = _write(self, TOKEN_DATA, buf->buf, buf->len);
//        if (r < 0) {
//            return r;
//        }
//    } else {
//        //  Use CMD25 to write multiple block
//        int r = block_cmd(self, 25, start_block, NULL, 0, true, true);
//        if (r < 0) {
//            return r;
//        }
//
//        uint8_t *ptr = buf->buf;
//        while (nblocks--) {
//            r = _write(self, TOKEN_CMD25, ptr, 512);
//            if (r < 0) {
//                return r;
//            }
//            ptr += 512;
//        }
//
//        cmd_nodata(self, TOKEN_STOP_TRAN, 0);
//    }
//    return 0;
//}
//
//int common_hal_sdcardio_sdcard_writeblocks(sdcardio_sdcard_obj_t *self, uint32_t start_block, mp_buffer_info_t *buf) {
//    common_hal_sdcardio_check_for_deinit(self);
//    if (buf->len % 512 != 0) {
//        mp_raise_ValueError(translate("Buffer length must be a multiple of 512"));
//    }
//    lock_and_configure_bus(self);
//    int r = writeblocks(self, start_block, buf);
//    extraclock_and_unlock_bus(self);
//    return r;
//}
