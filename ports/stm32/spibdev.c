/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017-2018 Damien P. George
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

#include "py/obj.h"
#include "py/mperrno.h"
#include "py/runtime.h"
#include "spi.h"
#include "drivers/memory/spiflash.h"

#include "irq.h"
#include "led.h"
#include "storage.h"
#include "pin.h"
#include "sdcardio/SDCard.h"

#if MICROPY_HW_ENABLE_STORAGE

int32_t spi_bdev_ioctl(spi_bdev_t *bdev, uint32_t op, uint32_t arg) {
    switch (op) {
        case BDEV_IOCTL_INIT:
            bdev->spiflash.config = (const mp_spiflash_config_t *)arg;
            mp_spiflash_init(&bdev->spiflash);
            bdev->flash_tick_counter_last_write = 0;
            return 0;

        case BDEV_IOCTL_IRQ_HANDLER:
            #if MICROPY_HW_SPIFLASH_ENABLE_CACHE
            if ((bdev->spiflash.flags & 1) && HAL_GetTick() - bdev->flash_tick_counter_last_write >= 1000) {
                mp_spiflash_cache_flush(&bdev->spiflash);
                led_state(PYB_LED_RED, 0); // indicate a clean cache with LED off
            }
            #endif
            return 0;

        case BDEV_IOCTL_SYNC:
            #if MICROPY_HW_SPIFLASH_ENABLE_CACHE
            if (bdev->spiflash.flags & 1) {
                uint32_t basepri = raise_irq_pri(IRQ_PRI_FLASH); // prevent cache flushing and USB access
                mp_spiflash_cache_flush(&bdev->spiflash);
                led_state(PYB_LED_RED, 0); // indicate a clean cache with LED off
                restore_irq_pri(basepri);
            }
            #endif
            return 0;

        case BDEV_IOCTL_BLOCK_ERASE: {
            uint32_t basepri = raise_irq_pri(IRQ_PRI_FLASH); // prevent cache flushing and USB access
            mp_spiflash_erase_block(&bdev->spiflash, arg * MP_SPIFLASH_ERASE_BLOCK_SIZE);
            restore_irq_pri(basepri);
            return 0;
        }
    }
    return -MP_EINVAL;
}

#if MICROPY_HW_SPIFLASH_ENABLE_CACHE
int spi_bdev_readblocks(spi_bdev_t *bdev, uint8_t *dest, uint32_t block_num, uint32_t num_blocks) {
    uint32_t basepri = raise_irq_pri(IRQ_PRI_FLASH); // prevent cache flushing and USB access
    mp_spiflash_cached_read(&bdev->spiflash, block_num * FLASH_BLOCK_SIZE, num_blocks * FLASH_BLOCK_SIZE, dest);
    restore_irq_pri(basepri);

    return 0;
}

int spi_bdev_writeblocks(spi_bdev_t *bdev, const uint8_t *src, uint32_t block_num, uint32_t num_blocks) {
    uint32_t basepri = raise_irq_pri(IRQ_PRI_FLASH); // prevent cache flushing and USB access
    int ret = mp_spiflash_cached_write(&bdev->spiflash, block_num * FLASH_BLOCK_SIZE, num_blocks * FLASH_BLOCK_SIZE, src);
    if (bdev->spiflash.flags & 1) {
        led_state(PYB_LED_RED, 1); // indicate a dirty cache with LED on
        bdev->flash_tick_counter_last_write = HAL_GetTick();
    }
    restore_irq_pri(basepri);

    return ret;
}
#endif // MICROPY_HW_SPIFLASH_ENABLE_CACHE

int spi_bdev_readblocks_raw(spi_bdev_t *bdev, uint8_t *dest, uint32_t block_num, uint32_t block_offset, uint32_t num_bytes) {
    uint32_t basepri = raise_irq_pri(IRQ_PRI_FLASH); // prevent cache flushing and USB access
    mp_spiflash_read(&bdev->spiflash, block_num * MP_SPIFLASH_ERASE_BLOCK_SIZE + block_offset, num_bytes, dest);
    restore_irq_pri(basepri);

    return 0;
}

int spi_bdev_writeblocks_raw(spi_bdev_t *bdev, const uint8_t *src, uint32_t block_num, uint32_t block_offset, uint32_t num_bytes) {
    uint32_t basepri = raise_irq_pri(IRQ_PRI_FLASH); // prevent cache flushing and USB access
    int ret = mp_spiflash_write(&bdev->spiflash, block_num * MP_SPIFLASH_ERASE_BLOCK_SIZE + block_offset, num_bytes, src);
    restore_irq_pri(basepri);

    return ret;
}
#endif

/*
*Micropython bindings
*/

typedef struct _pyb_spibdev_obj_t {
    mp_obj_base_t base;
    spi_bdev_t *bdev;
} pyb_spibdev_obj_t;

STATIC void pyb_spibdev_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    // pyb_flash_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // if (self == &pyb_flash_obj) {
    //     mp_printf(print, "Flash()");
    // } else {
    //     mp_printf(print, "Flash(start=%u, len=%u)", self->start, self->len);
    // }
    return;
}

STATIC mp_obj_t pyb_spibdev_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    // machine_hard_spi_obj_t *spi = MP_OBJ_TO_PTR(all_args[0]);
    const spi_t *spi = spi_from_mp_obj(all_args[0]);
    pin_obj_t *pin = MP_OBJ_TO_PTR(all_args[1]);
	sdcardio_sdcard_obj_t *self = m_new_obj(sdcardio_sdcard_obj_t);
    common_hal_sdcardio_sdcard_construct(self, spi, pin, 500);
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t pyb_spibdev_readblocks(size_t n_args, const mp_obj_t *args) {
    pyb_spibdev_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint8_t buff;
    uint32_t block_num = mp_obj_get_int(args[1]);
    uint32_t num_blocks = mp_obj_get_int(args[2]);
    spi_bdev_readblocks(self->bdev, &buff, block_num, num_blocks);
    return MP_OBJ_NEW_SMALL_INT(buff);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_spibdev_readblocks_obj, 3, 4, pyb_spibdev_readblocks);

STATIC mp_obj_t pyb_spibdev_writeblocks(size_t n_args, const mp_obj_t *args) {
    pyb_spibdev_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    const uint8_t *src = MP_OBJ_TO_PTR(args[1]);
    uint32_t block_num = mp_obj_get_int(args[2]);
    uint32_t num_blocks = mp_obj_get_int(args[3]);
    int ret = spi_bdev_writeblocks(self->bdev, src, block_num, num_blocks);
    return MP_OBJ_NEW_SMALL_INT(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_spibdev_writeblocks_obj, 3, 4, pyb_spibdev_writeblocks);

STATIC mp_obj_t pyb_spibdev_ioctl(mp_obj_t self_in, mp_obj_t cmd_in, mp_obj_t arg_in) {
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_spibdev_ioctl_obj, pyb_spibdev_ioctl);

STATIC const mp_rom_map_elem_t pyb_spibdev_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_readblocks), MP_ROM_PTR(&pyb_spibdev_readblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeblocks), MP_ROM_PTR(&pyb_spibdev_writeblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_ioctl), MP_ROM_PTR(&pyb_spibdev_ioctl_obj) },
};

STATIC MP_DEFINE_CONST_DICT(pyb_spibdev_locals_dict, pyb_spibdev_locals_dict_table);

const mp_obj_type_t pyb_spibdev_type = {
    { &mp_type_type },
    .name = MP_QSTR_Spibdev,
    .print = pyb_spibdev_print,
    .make_new = pyb_spibdev_make_new,
    .locals_dict = (mp_obj_dict_t *)&pyb_spibdev_locals_dict,
};
