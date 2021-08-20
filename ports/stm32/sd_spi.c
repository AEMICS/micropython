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
#include <sd_spi.h>
#include "pin.h"
#include "py/mperrno.h"
#include <string.h>
#include "storage.h"

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

bool spi_write(sd_spi_obj_t *self, const uint8_t *data, size_t len)
{
    spi_transfer(self->bus, len, data, NULL, SPI_TRANSFER_TIMEOUT(len));

    return true;
}

bool spi_read(sd_spi_obj_t *self, uint8_t *data, size_t len, uint8_t write_value)
{
	memset(data, write_value, len);
    spi_transfer(self->bus, len, data, data, SPI_TRANSFER_TIMEOUT(len));
    return true;
}

STATIC void clock_card(sd_spi_obj_t *self, int bytes)
{
    uint8_t buf[] = {0xff};
    mp_hal_pin_high(self->cs);
    for (int i = 0; i < bytes; i++)
    {
        spi_write(self, buf, 1);
    }
}

STATIC void extraclock_and_unlock_bus(sd_spi_obj_t *self) {
    clock_card(self, 1);
    mp_hal_pin_high(self->cs);
}

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
STATIC void wait_for_ready(sd_spi_obj_t *self)
{
	mp_uint_t deadline = mp_hal_ticks_us() + READY_TIMEOUT_US;
    while (mp_hal_ticks_us() < deadline)
    {
        uint8_t b;
        spi_read(self, &b, 1, 0xff);
        if (b == 0xff)
        {
            break;
        }
    }
}
//// In Python API, defaults are response=None, data_block=True, wait=True
STATIC int cmd(sd_spi_obj_t *self, int cmd, int arg, void *response_buf, size_t response_len, bool data_block, bool wait)
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
    spi_write(self, cmdbuf, sizeof(cmdbuf));

    // Wait for the response (response[7] == 0)
    bool response_received = false;
    for (int i = 0; i < CMD_TIMEOUT; i++)
    {
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
                spi_read(self, cmdbuf + 1, 1, 0xff);
            } while (cmdbuf[1] != 0xfe);
        }

        spi_read(self, response_buf, response_len, 0xff);

        if (data_block)
        {
            // Read and discard the CRC-CCITT checksum
            spi_read(self, cmdbuf + 1, 2, 0xff);
        }
    }

    return cmdbuf[0];
}

STATIC int block_cmd(sd_spi_obj_t *self, int cmd_, int block, void *response_buf, size_t response_len, bool data_block, bool wait)
{
    return cmd(self, cmd_, block * self->cdv, response_buf, response_len, true, true);
}

 STATIC bool cmd_nodata(sd_spi_obj_t *self, int cmd, int response)
 {
     uint8_t cmdbuf[2] = {cmd, 0xff};

     spi_write(self, cmdbuf, sizeof(cmdbuf));
     // Wait for the response (response[7] == response)
     for (int i = 0; i < CMD_TIMEOUT; i++)
     {
         spi_read(self, cmdbuf, 1, 0xff);
         if (cmdbuf[0] == response)
         {
             return 0;
         }
     }
     return -EIO;
 }

 STATIC const int init_card_v1(sd_spi_obj_t *self)
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

 STATIC const int init_card_v2(sd_spi_obj_t *self) {
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

STATIC const int init_card(sd_spi_obj_t *self)
{
    clock_card(self, 16);

    //  CMD0: init card: should return _R1_IDLE_STATE (allow 5 attempts)
    {
        bool reached_idle_state = false;
        for (int i = 0; i < 5; i++)
        {
            mp_hal_pin_low(self->cs);
            if (cmd(self, 0, 0, NULL, 0, true, true) == R1_IDLE_STATE)
            {
                reached_idle_state = true;
                break;
            }
            clock_card(self, 1);
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
        mp_hal_pin_low(self->cs);
        int response = cmd(self, 8, 0x1AA, rb7, sizeof(rb7), false, true);
        clock_card(self, 1);
        mp_hal_pin_low(self->cs);
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
        clock_card(self, 1);
        mp_hal_pin_low(self->cs);
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
        int response = cmd(self, 16, 512, NULL, 0, true, true);
        clock_card(self, 1);
        mp_hal_pin_low(self->cs);
        if (response != 0)
        {
            // return translate("can't set 512 block size");
            return CANT_SET_512_BLOCK_SIZE;
        }
    }

    return 0;
}

void sd_spi_construct(sd_spi_obj_t *self, const spi_t *bus, pin_obj_t *cs, int baudrate)
{
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
    init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; //SPI_BAUDRATEPRESCALER_2; // clock freq = f_PCLK / this_prescale_value; Wiz820i can do up to 80MHz
    init->FirstBit = SPI_FIRSTBIT_MSB;
    init->TIMode = SPI_TIMODE_DISABLED;
    init->CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    init->CRCPolynomial = 7; // unused
    spi_init(self->bus, false);

    //    lock_bus_or_throw(self);
    mp_hal_pin_low(self->cs);
    const int result = init_card(self);
    extraclock_and_unlock_bus(self);

    if (result != 0)
    {
        self->cs = NULL;
        mp_raise_msg(&mp_type_OSError, NULL);
    }

    self->baudrate = baudrate;
}
//
//void common_hal_sdcardio_sdcard_deinit(sd_spi_obj_t *self) {
//    if (!self->bus) {
//        return;
//    }
//    self->bus = 0;
//    common_hal_digitalio_digitalinout_deinit(&self->cs);
//}

void sd_spi_check_for_deinit(sd_spi_obj_t *self)
{
    if (!self->bus)
    {
        mp_raise_msg(&mp_type_OSError, NULL);
        //    raise_deinited_error();
    }
}

int sd_spi_get_blockcount(sd_spi_obj_t *self)
{
    sd_spi_check_for_deinit(self);
    return self->sectors;
}

int readinto(sd_spi_obj_t *self, void *buf, size_t size)
{
    uint8_t aux[2] = {0, 0};
    while (aux[0] != 0xfe)
    {
        spi_read(self, aux, 1, 0xff);
    }

    spi_read(self, buf, size, 0xff);

    // Read checksum and throw it away
    spi_read(self, aux, sizeof(aux), 0xff);
    return 0;
}

int readblocks(sd_spi_obj_t *self, uint32_t start_block, mp_buffer_info_t *buf)
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

int sd_spi_readblocks(sd_spi_obj_t *self, uint32_t start_block, mp_buffer_info_t *buf) {
    sd_spi_check_for_deinit(self);
    if (buf->len % 512 != 0) {
    	mp_printf(&mp_plat_print, "Buffer length must be a multiple of 512\n");
    	mp_raise_msg(&mp_type_OSError, NULL);
    }

//    lock_and_configure_bus(self);
    mp_hal_pin_low(self->cs);
    int r = readblocks(self, start_block, buf);
    extraclock_and_unlock_bus(self);
    return r;
}

STATIC int _write(sd_spi_obj_t *self, uint8_t token, void *buf, size_t size) {
    wait_for_ready(self);

    uint8_t cmd[2];
    cmd[0] = token;

    spi_write(self, cmd, 1);
    spi_write(self, buf, size);

    cmd[0] = cmd[1] = 0xff;
    spi_write(self, cmd, 2);

    // Check the response
    // This differs from the traditional adafruit_sdcard handling,
    // but adafruit_sdcard also ignored the return value of SDCard._write(!)
    // so nobody noticed
    //
    //
    // Response is as follows:
    //  x x x 0 STAT 1
    //  7 6 5 4 3..1 0
    // with STATUS 010 indicating "data accepted", and other status bit
    // combinations indicating failure.
    // In practice, I was seeing cmd[0] as 0xe5, indicating success
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        spi_read(self, cmd, 1, 0xff);
        DEBUG_PRINT("i=%02d cmd[0] = 0x%02x\n", i, cmd[0]);
        if ((cmd[0] & 0b00010001) == 0b00000001) {
            if ((cmd[0] & 0x1f) != 0x5) {
                return -EIO;
            } else {
                break;
            }
        }
    }

    // Wait for the write to finish
    do {
        spi_read(self, cmd, 1, 0xff);
    } while (cmd[0] == 0);

    // Success
    return 0;
}

STATIC int writeblocks(sd_spi_obj_t *self, uint32_t start_block, mp_buffer_info_t *buf) {
    sd_spi_check_for_deinit(self);
    uint32_t nblocks = buf->len / 512;
    if (nblocks == 1) {
        //  Use CMD24 to write a single block
        int r = block_cmd(self, 24, start_block, NULL, 0, true, true);
        if (r < 0) {
            return r;
        }
        r = _write(self, TOKEN_DATA, buf->buf, buf->len);
        if (r < 0) {
            return r;
        }
    } else {
        //  Use CMD25 to write multiple block
        int r = block_cmd(self, 25, start_block, NULL, 0, true, true);
        if (r < 0) {
            return r;
        }

        uint8_t *ptr = buf->buf;
        while (nblocks--) {
            r = _write(self, TOKEN_CMD25, ptr, 512);
            if (r < 0) {
                return r;
            }
            ptr += 512;
        }

        cmd_nodata(self, TOKEN_STOP_TRAN, 0);
    }
    return 0;
}

int sd_spi_writeblocks(sd_spi_obj_t *self, uint32_t start_block, mp_buffer_info_t *buf) {
    sd_spi_check_for_deinit(self);
    if (buf->len % 512 != 0) {
    	mp_printf(&mp_plat_print, "Buffer length must be a multiple of 512\n");
        mp_raise_ValueError(NULL);
    }
    mp_hal_pin_low(self->cs);
    int r = writeblocks(self, start_block, buf);
    mp_hal_pin_high(self->cs);
    return r;
}

/*
*Micropython bindings
*/

STATIC void pyb_sd_spi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    // pyb_flash_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // if (self == &pyb_flash_obj) {
    //     mp_printf(print, "Flash()");
    // } else {
    //     mp_printf(print, "Flash(start=%u, len=%u)", self->start, self->len);
    // }
    return;
}

STATIC mp_obj_t pyb_sd_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    const spi_t *spi = spi_from_mp_obj(all_args[0]);
    pin_obj_t *pin = MP_OBJ_TO_PTR(all_args[1]);
    sd_spi_obj_t *self = m_new_obj(sd_spi_obj_t);
	self->base.type = &pyb_sd_spi_type;
    sd_spi_construct(self, spi, pin, 500);
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t pyb_sd_spi_readblocks(mp_obj_t self_in, mp_obj_t start_block_in, mp_obj_t buf_in) {
	sd_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t block_num = mp_obj_get_int(start_block_in);
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_WRITE);

    int result = sd_spi_readblocks(self, block_num, &buf);
    if (result < 0) {
        mp_raise_OSError(-result);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sd_spi_readblocks_obj, pyb_sd_spi_readblocks);

STATIC mp_obj_t pyb_sd_spi_writeblocks(mp_obj_t self_in, mp_obj_t start_block_in, mp_obj_t buf_in) {
	sd_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);

    uint32_t block_num = mp_obj_get_int(start_block_in);
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_WRITE);

    int result = sd_spi_writeblocks(self, block_num, &buf);
    if (result < 0) {
        mp_raise_OSError(-result);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sd_spi_writeblocks_obj, pyb_sd_spi_writeblocks);

STATIC mp_obj_t pyb_sd_spi_ioctl(mp_obj_t self_in, mp_obj_t cmd_in, mp_obj_t arg_in) {
//    return mp_const_none;
	return 0;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sd_spi_ioctl_obj, pyb_sd_spi_ioctl);

STATIC const mp_rom_map_elem_t pyb_sd_spi_locals_dict_table[] = {
     { MP_ROM_QSTR(MP_QSTR_readblocks), MP_ROM_PTR(&pyb_sd_spi_readblocks_obj) },
     { MP_ROM_QSTR(MP_QSTR_writeblocks), MP_ROM_PTR(&pyb_sd_spi_writeblocks_obj) },
     { MP_ROM_QSTR(MP_QSTR_ioctl), MP_ROM_PTR(&pyb_sd_spi_ioctl_obj) },
};

STATIC MP_DEFINE_CONST_DICT(pyb_sd_spi_locals_dict, pyb_sd_spi_locals_dict_table);

const mp_obj_type_t pyb_sd_spi_type = {
    { &mp_type_type },
    .name = MP_QSTR_sd_spi,
    .print = pyb_sd_spi_print,
    .make_new = pyb_sd_spi_make_new,
    .locals_dict = (mp_obj_dict_t *)&pyb_sd_spi_locals_dict,
};
