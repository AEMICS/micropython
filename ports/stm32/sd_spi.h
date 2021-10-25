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

#ifndef MICROPY_INCLUDED_STM32_SD_SPI_H
#define MICROPY_INCLUDED_STM32_SD_SPI_H

#include "py/obj.h"
#include "py/runtime.h"
#include "py/objarray.h"
#include "errno.h"
#include "py/mphal.h"
#include "extmod/vfs_fat.h"
#include "pin.h"
#include "spi.h"

typedef struct {
    // const mp_hal_pin_obj_t *cs;
// #if defined(MICROPY_HW_SD_SPI_CD)
    // const mp_hal_pin_obj_t *cd;
// #endif
    const spi_t *bus;
    int cdv;
    int baudrate;
    uint32_t sectors;
} sd_spi_obj_t;


typedef struct _pyb_sd_spi_obj_t {
    mp_obj_base_t base;
    uint32_t start; // in bytes
    uint32_t len; // in bytes
    bool use_native_block_size;
} pyb_sd_spi_obj_t;

// this is a fixed size and should not be changed
#define SDCARD_BLOCK_SIZE (512)
int sd_spi_ioctl(int cmd);
void sd_spi_construct();
void sd_spi_deinit();
void sd_spi_check_for_deinit();
int sd_spi_get_blockcount();
bool sd_spi_card_inserted();
int sd_spi_readblocks(uint8_t *dest, uint32_t start_block, uint32_t num_blocks);
int sd_spi_writeblocks(uint8_t *src, uint32_t start_block, uint32_t num_blocks);
void sd_spi_init_vfs(fs_user_mount_t *vfs, int part);

extern const struct _mp_obj_type_t pyb_sd_spi_type;
extern const struct _mp_obj_base_t pyb_sd_spi_obj;


#endif // MICROPY_INCLUDED_STM32_SD_SPI_H
