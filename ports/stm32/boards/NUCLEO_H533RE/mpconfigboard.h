#define MICROPY_HW_BOARD_NAME "NUCLEO_H533RE"
#define MICROPY_HW_MCU_NAME "STM32H533RE"

#define MICROPY_PY_PYB_LEGACY (0)
#define MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE (1)
#define MICROPY_HW_ENABLE_RTC (1)
#define MICROPY_HW_ENABLE_RNG (1)
#define MICROPY_HW_ENABLE_ADC (1)
#define MICROPY_HW_ENABLE_DAC (1)
#define MICROPY_HW_ENABLE_USB (1)
#define MICROPY_HW_HAS_SWITCH (1)
#define MICROPY_HW_HAS_FLASH (1)

#define MICROPY_HW_CLK_USE_HSE (1)
// The board has a 24MHz oscillator, the following gives 250MHz CPU speed
#define MICROPY_HW_CLK_USE_BYPASS (0)
#define MICROPY_HW_CLK_PLLM (6)
#define MICROPY_HW_CLK_PLLN (125)
#define MICROPY_HW_CLK_PLLP (2)
#define MICROPY_HW_CLK_PLLQ (2)
#define MICROPY_HW_CLK_PLLR (2)
#define MICROPY_HW_CLK_PLLVCI_LL (LL_RCC_PLLINPUTRANGE_4_8)
#define MICROPY_HW_CLK_PLLVCO_LL (LL_RCC_PLLVCORANGE_WIDE)
#define MICROPY_HW_CLK_PLLFRAC (0)

// PLL3 with Q output at 48MHz for USB, taken from HSE
// #define MICROPY_HW_CLK_USE_PLL3_FOR_USB
// #define MICROPY_HW_CLK_PLL3M (8)
// #define MICROPY_HW_CLK_PLL3N (48)
// #define MICROPY_HW_CLK_PLL3P (2)
// #define MICROPY_HW_CLK_PLL3Q (3)
// #define MICROPY_HW_CLK_PLL3R (2)
// #define MICROPY_HW_CLK_PLL3FRAC (0)
// #define MICROPY_HW_CLK_PLL3VCI_LL (LL_RCC_PLLINPUTRANGE_1_2)
// #define MICROPY_HW_CLK_PLL3VCO_LL (LL_RCC_PLLVCORANGE_MEDIUM)
#define MICROPY_HW_CLK_USE_HSI48 (1)

// 5 wait states, according to Table 37, Reference Manual (RM0481 Rev 1)
#define MICROPY_HW_FLASH_LATENCY FLASH_LATENCY_5

// There is an external 32kHz oscillator
#define MICROPY_HW_RTC_USE_LSE (1)

// UART config
#define MICROPY_HW_UART1_TX (pin_B14) // ARD_D1_TX
#define MICROPY_HW_UART1_RX (pin_B15) // ARD_D0_RX
#define MICROPY_HW_UART2_TX (pin_A2)  // VCP TX STlink
#define MICROPY_HW_UART2_RX (pin_A3)  // VCP RX STlink
// #define MICROPY_HW_UART3_TX (pin_A4) // n.c.
// #define MICROPY_HW_UART3_RX (pin_A3) // .. TX2

// Connect REPL to UART2 which is provided on ST-Link USB interface
#define MICROPY_HW_UART_REPL PYB_UART_2
#define MICROPY_HW_UART_REPL_BAUD 115200

// I2C buses
#define MICROPY_HW_I2C1_SCL (pin_B6) // Arduino Connector CN5-Pin10 (D15)
#define MICROPY_HW_I2C1_SDA (pin_B7) // Arduino Connector CN5-Pin9 (D14)

// SPI buses
// PD14 according to datasheet not working as SPI1_NSS, have to use as GPIO, not
// as AF
#define MICROPY_HW_SPI1_NSS (pin_C9)  // Arduino Connector CN5-Pin3 (D10)
#define MICROPY_HW_SPI1_SCK (pin_A5)  // Arduino Connector CN5-Pin6 (D13)
#define MICROPY_HW_SPI1_MISO (pin_A6) // Arduino Connector CN5-Pin5 (D12)
#define MICROPY_HW_SPI1_MOSI (pin_A7) // Arduino Connector CN5-Pin4 (D11)

// USRSW is pulled low. Pressing the button makes the input go high.
#define MICROPY_HW_USRSW_PIN (pin_C13)
#define MICROPY_HW_USRSW_PULL (GPIO_NOPULL)
#define MICROPY_HW_USRSW_EXTI_MODE (GPIO_MODE_IT_RISING)
#define MICROPY_HW_USRSW_PRESSED (1)

// LEDs
#define MICROPY_HW_LED1 (pin_A5) // Green
#define MICROPY_HW_LED_ON(pin) (mp_hal_pin_high(pin))
#define MICROPY_HW_LED_OFF(pin) (mp_hal_pin_low(pin))

// USB config
#define MICROPY_HW_USB_FS (1)
#define MICROPY_HW_USB_MAIN_DEV (USB_PHY_FS_ID)
