#define MICROPY_HW_BOARD_NAME       "AEMICS PYglet with Internal File System"
#define MICROPY_HW_MCU_NAME         "STM32G473"
#define MICROPY_HW_FLASH_FS_LABEL   "PYglet"

#define MICROPY_HW_USB_FS           (0)
#define MICROPY_HW_ENABLE_RTC       (0)
#define MICROPY_HW_ENABLE_RNG       (0)
#define MICROPY_HW_ENABLE_ADC       (1)
#define MICROPY_HW_ENABLE_DAC       (1)  // A4, A5
#define MICROPY_HW_ENABLE_USB       (0)  // A12 (dp), A11 (dm)
#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_LED          (1)
#define MICROPY_HW_HAS_FLASH        (0)  // QSPI extflash not mounted
#define MICROPY_HW_UART_REPL        (PYB_UART_4)
#define MICROPY_HW_UART_REPL_BAUD   (115200)

#define MICROPY_BOARD_EARLY_INIT    board_early_init
void board_early_init(void);

#define MICROPY_HW_USB_MANUFACTURER_STRING    "AEMICS"
#define MICROPY_HW_USB_PRODUCT_FS_STRING   MICROPY_HW_BOARD_NAME

// ports/stm32/mpconfigport.h
#define MICROPY_PY_LWIP      (0)  // Geen ETH
#define MICROPY_PY_USSL      (0)
#define MICROPY_SSL_MBEDTLS  (0)
#define MICROPY_PY_UASYNCIO  (0)
#define MICROPY_PY_UZLIB     (0)
#define MICROPY_PY_UJSON     (1)
#define MICROPY_PY_URE       (0)
#define MICROPY_PY_FRAMEBUF  (0)
#define MICROPY_PY_USOCKET   (0)
#define MICROPY_PY_NETWORK   (0)
#define MICROPY_PERSISTENT_CODE_LOAD (1)

// The board has an 24MHz HSE, the following gives 170MHz CPU speed
#define MICROPY_HW_CLK_USE_LSE      (0)
#define MICROPY_HW_CLK_USE_HSE      (0)
#define MICROPY_HW_CLK_USE_HSI (1)
#define MICROPY_HW_CLK_PLLM (6)
#define MICROPY_HW_CLK_PLLN (85)
#define MICROPY_HW_CLK_PLLP (2)
#define MICROPY_HW_CLK_PLLQ (2)
#define MICROPY_HW_CLK_PLLR (2)

// 4 wait states
#define MICROPY_HW_FLASH_LATENCY    FLASH_LATENCY_8

// UART config
#define MICROPY_HW_UART4_TX         (pin_C10)
#define MICROPY_HW_UART4_RX         (pin_C11)

// xSPI
//SPI2
#define MICROPY_HW_SPI2_NSS         (pin_B12)
#define MICROPY_HW_SPI2_SCK         (pin_B13)
#define MICROPY_HW_SPI2_MISO        (pin_B14)
#define MICROPY_HW_SPI2_MOSI        (pin_B15)

// USRSW has pullup, pressing the switch makes the input go low
#define MICROPY_HW_USRSW_PIN        (pin_B8)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLDOWN)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_RISING)
#define MICROPY_HW_USRSW_PRESSED    (1)

// LED
#define MICROPY_HW_LED1             (pin_B7) // Red
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_low(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_high(pin))

