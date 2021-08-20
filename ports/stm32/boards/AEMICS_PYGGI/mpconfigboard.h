/* This file is part of the MicroPython project, http://micropython.org/
 * MIT License; Copyright (c) 2019 Damien P. George
 */

#define MICROPY_HW_BOARD_NAME       "AEPYM"
#define MICROPY_HW_MCU_NAME         "STM32WB55VGQ6"
#ifdef USBD_MANUFACTURER_STRING
#undef USBD_MANUFACTURER_STRING
#endif
#define USBD_MANUFACTURER_STRING    "AEMICS"
#ifdef USBD_PRODUCT_FS_STRING
#undef USBD_PRODUCT_FS_STRING
#endif
#define USBD_PRODUCT_FS_STRING   MICROPY_HW_BOARD_NAME

#define MICROPY_PY_PYB_LEGACY       (0)

#define MICROPY_PY_STM              (1)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_ADC       (1)
#define MICROPY_HW_ENABLE_USB       (1)
#define MICROPY_HW_ENABLE_TIMER     (1)
#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (1)
#define MICROPY_HW_FLASH_FS_LABEL   "aemics"
#define MICROPY_HW_STM32WB_FLASH_SYNCRONISATION (1)

// External HSE is 32MHz
// Resulting core frequency is 64MHz:
#define MICROPY_HW_CLK_USE_HSE      (1)
#define MICROPY_HW_CLK_PLLM         (8)
#define MICROPY_HW_CLK_PLLN         (32)
#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLR (RCC_PLLR_DIV2)
#define MICROPY_HW_CLK_PLLQ (RCC_PLLQ_DIV2)
#define MICROPY_HW_CLK_PLLSAIN      (24)
#define MICROPY_HW_CLK_PLLSAIP (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLSAIQ (RCC_PLLQ_DIV2)
#define MICROPY_HW_CLK_PLLSAIR (RCC_PLLR_DIV2)

// There is an external 32kHz oscillator
#define RTC_ASYNCH_PREDIV           (0)
#define RTC_SYNCH_PREDIV            (0x7fff)
#define MICROPY_HW_RTC_USE_LSE      (1)
#define MICROPY_HW_RTC_USE_US       (1)

// I2C buses
#define MICROPY_HW_I2C1_SCL         (pin_B6)
#define MICROPY_HW_I2C1_SDA         (pin_B7)
#define MICROPY_HW_I2C3_SCL         (pin_B13)
#define MICROPY_HW_I2C3_SDA         (pin_B14)

// SPI buses
#define MICROPY_HW_SPI1_SCK         (pin_B3)
#define MICROPY_HW_SPI1_MISO        (pin_B4)
#define MICROPY_HW_SPI1_MOSI        (pin_B5)
#define MICROPY_HW_SPI1_NSS         (pin_B2)
#define MICROPY_HW_SPI2_SCK         (pin_D1)
#define MICROPY_HW_SPI2_MISO        (pin_D3)
#define MICROPY_HW_SPI2_MOSI        (pin_D4)
#define MICROPY_HW_SPI2_NSS         (pin_D0)

// UART config
#define MICROPY_HW_UART1_RX         (pin_A10)
#define MICROPY_HW_UART1_TX         (pin_A9)

// User switch; pressing the button makes the input go high
#define MICROPY_HW_USRSW_PIN        (pin_C13)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLDOWN)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_RISING)
#define MICROPY_HW_USRSW_PRESSED    (1)

// LEDs
#define MICROPY_HW_LED_INVERTED     (1)
#define MICROPY_HW_LED1             (pin_A1) // red TIM2_CH2
#define MICROPY_HW_LED2             (pin_A15) // green TIM2_CH1
#define MICROPY_HW_LED3             (pin_B11) // blue TIM2_CH4
#define MICROPY_HW_LED1_PWM         { TIM2, 2, TIM_CHANNEL_2, GPIO_AF1_TIM2 }
#define MICROPY_HW_LED2_PWM         { TIM2, 2, TIM_CHANNEL_1, GPIO_AF1_TIM2 }
#define MICROPY_HW_LED3_PWM         { TIM2, 2, TIM_CHANNEL_4, GPIO_AF1_TIM2 }
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_low(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_high(pin))

// USB config
#define MICROPY_HW_USB_FS           (1)

// Bluetooth config
#define MICROPY_HW_BLE_UART_ID       (0)
#define MICROPY_HW_BLE_UART_BAUDRATE (115200)
