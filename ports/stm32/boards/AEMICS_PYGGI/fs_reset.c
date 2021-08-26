#include "py/runtime.h"
#include "py/mperrno.h"
#include "extmod/vfs_fat.h"
#include "systick.h"
#include "led.h"
#include "storage.h"
#include "factoryreset.h"
#include "fs_reset.h"

static const char fresh_boot_py[] =
    "# boot.py -- run on boot-up\r\n"
    "# can run arbitrary Python, but best to keep it minimal\r\n"
    "\r\n"
    "import machine\r\n"
    "import pyb\r\n"
    "pyb.country('US') # ISO 3166-1 Alpha-2 code, eg US, GB, DE, AU\r\n"
    "#pyb.main('main.py') # main script to run after this one\r\n"
#if MICROPY_HW_ENABLE_USB
    "#pyb.usb_mode('VCP+MSC') # act as a serial and a storage device\r\n"
    "#pyb.usb_mode('VCP+HID') # act as a serial device and a mouse\r\n"
#endif
;

static const char fresh_main_py[] =
    "# main.py -- put your code here!\r\n"
;

#if MICROPY_HW_ENABLE_USB
static const char fresh_pybcdc_inf[] =
#include "genhdr/pybcdc_inf.h"
;

static const char fresh_readme_txt[] =
    "Welcome to your PYggi board\r\n"
    "This is a MicroPython module\r\n"
    "\r\n"
    "You can get started right away by writing your Python code in 'main.py'.\r\n"
    "\r\n"
    "For a serial prompt:\r\n"
    " - Windows: you need to go to 'Device manager', right click on the unknown device,\r\n"
    "   then update the driver software, using the 'pybcdc.inf' file found on this drive.\r\n"
    "   Then use a terminal program like Hyperterminal or putty.\r\n"
    " - Mac OS X: use the command: screen /dev/tty.usbmodem*\r\n"
    " - Linux: use the command: screen /dev/ttyACM0\r\n"
    "\r\n"
    "Please visit https://www.aemics.nl/pyg/ for more information, documentation and tutorials.\r\n"
;
#endif

typedef struct _factory_file_t {
    const char *name;
    size_t len;
    const char *data;
} factory_file_t;

static const factory_file_t factory_files[] = {
    {"boot.py", sizeof(fresh_boot_py) - 1, fresh_boot_py},
    {"main.py", sizeof(fresh_main_py) - 1, fresh_main_py},
    #if MICROPY_HW_ENABLE_USB
    {"pybcdc.inf", sizeof(fresh_pybcdc_inf) - 1, fresh_pybcdc_inf},
    {"README.txt", sizeof(fresh_readme_txt) - 1, fresh_readme_txt},
    #endif
};

void factory_reset_make_files(FATFS *fatfs) {
    for (int i = 0; i < MP_ARRAY_SIZE(factory_files); ++i) {
        const factory_file_t *f = &factory_files[i];
        FIL fp;
        FRESULT res = f_open(fatfs, &fp, f->name, FA_WRITE | FA_CREATE_ALWAYS);
        if (res == FR_OK) {
            UINT n;
            f_write(&fp, f->data, f->len, &n);
            f_close(&fp);
        }
    }
}