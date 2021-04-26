DEBUG = 0

# MCU settings
MCU_SERIES = g4
CMSIS_MCU = STM32G473xx
MICROPY_FLOAT_IMPL = double
AF_FILE = boards/stm32g473_af.csv
LD_FILES = boards/AEMICS_51044500/stm32g473.ld boards/common_basic.ld
FROZEN_MANIFEST = 0
