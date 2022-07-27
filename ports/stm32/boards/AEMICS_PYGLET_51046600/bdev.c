/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Damien P. George
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

#include "storage.h"
#include "qspi.h"
#include "py/mphal.h"

#if MICROPY_HW_SPIFLASH_ENABLE_CACHE
// Shared cache for first and second SPI block devices
STATIC mp_spiflash_cache_t spi_bdev_cache;
#endif

// First external SPI flash uses software QSPI interface

STATIC const mp_soft_qspi_obj_t soft_qspi_bus = {
    .cs = MICROPY_HW_QSPIFLASH_CS,
    .clk = MICROPY_HW_QSPIFLASH_SCK,
    .io0 = MICROPY_HW_QSPIFLASH_IO0,
    .io1 = MICROPY_HW_QSPIFLASH_IO1,
    .io2 = MICROPY_HW_QSPIFLASH_IO2,
    .io3 = MICROPY_HW_QSPIFLASH_IO3,
};

const mp_spiflash_config_t spiflash_config = {
    .bus_kind = MP_SPIFLASH_BUS_QSPI,
    .bus.u_qspi.data = (void*)&soft_qspi_bus,
    .bus.u_qspi.proto = &mp_soft_qspi_proto,
    #if MICROPY_HW_SPIFLASH_ENABLE_CACHE
    .cache = &spi_bdev_cache,
    #endif
};

spi_bdev_t spi_bdev;

void board_early_init(void) {
    qspi_init();
    qspi_memory_map();
}

//User and read protection, @ 0x1FFF 7800
const uint32_t optionbytes1  __attribute__(( section(".optionbytes1") )) = 0xFBEFF8AA; // default 0xFFEFF8AA
const uint32_t optionbytes1n  __attribute__(( section(".optionbytes1n") )) = ~optionbytes1;
//PCROP1 Start address, @ 0x1FFF 7808
const uint32_t optionbytes2  __attribute__(( section(".optionbytes2") )) = 0xFFFFFFFF;
const uint32_t optionbytes2n  __attribute__(( section(".optionbytes2n") )) = ~optionbytes2;
//PCROP1 End address, @ 0x1FFF 7810
const uint32_t optionbytes3  __attribute__(( section(".optionbytes3") )) = 0x00FF0000;
const uint32_t optionbytes3n  __attribute__(( section(".optionbytes3n") )) = ~optionbytes3;
//WRP1 Area A address, @ 0x1FFF 7818
const uint32_t optionbytes4  __attribute__(( section(".optionbytes4") )) = 0xFF00FFFF;
const uint32_t optionbytes4n  __attribute__(( section(".optionbytes4n") )) = ~optionbytes4;
//WRP2 Area A address, @ 0x1FFF 7820
const uint32_t optionbytes5  __attribute__(( section(".optionbytes5") )) = 0xFF00FFFF;
const uint32_t optionbytes5n  __attribute__(( section(".optionbytes5n") )) = ~optionbytes5;
//Securable memory area Bank 1, @ 0x1FFF 7828
const uint32_t optionbytes6  __attribute__(( section(".optionbytes6") )) = 0xFF00FF00;
const uint32_t optionbytes6n  __attribute__(( section(".optionbytes6n") )) = ~optionbytes6;

//PCROP2 Start address, @ 0x1FFFF808
const uint32_t optionbytes7  __attribute__(( section(".optionbytes7") )) = 0xFFFFFFFF;
const uint32_t optionbytes7n  __attribute__(( section(".optionbytes7n") )) = ~optionbytes7;
//PCROP2 End address, @ 0x1FFF F810
const uint32_t optionbytes8  __attribute__(( section(".optionbytes8") )) = 0x00FF0000;
const uint32_t optionbytes8n  __attribute__(( section(".optionbytes8n") )) = ~optionbytes8;
//WRP1 Area B address, @ 0x1FFF F818
const uint32_t optionbytes9  __attribute__(( section(".optionbytes9") )) = 0xFF00FFFF;
const uint32_t optionbytes9n  __attribute__(( section(".optionbytes9n") )) = ~optionbytes9;
//WRP2 Area B address, @ 0x1FFF F820
const uint32_t optionbytes10 __attribute__(( section(".optionbytes10") )) = 0xFF00FFFF;
const uint32_t optionbytes10n __attribute__(( section(".optionbytes10n") )) = ~optionbytes10;
//Securable memory area Bank 2, @ 0x1FFF F828
const uint32_t optionbytes11 __attribute__(( section(".optionbytes11") )) = 0xFF00FF00;
const uint32_t optionbytes11n __attribute__(( section(".optionbytes11n") )) = ~optionbytes11;
