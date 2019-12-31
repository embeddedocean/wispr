/**
 * WISPR V2.0 PINS Definitions
 */

#ifndef _WISPR_V2_0_BOARD_H
#define _WISPR_V2_0_BOARD_H

#include <conf_board.h>
#include <stdint.h>
#include <sam4sd32b.h>

extern void board_init(void);
extern void board_gpio_init(void);
extern uint32_t board_uart_init(int port, uint32_t baud);
extern uint32_t board_console_uart_init(void);
extern uint32_t board_wdt_init(uint32_t wdt_msec);
extern void display_reset_reason(void);

//! Name string macro
#define BOARD_NAME    "WISPR V2.0"

// Console UART
#define BOARD_CONSOLE_UART            UART1
#define BOARD_CONSOLE_UART_BAUDRATE   115200

//! Resonator definitions
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS  (12000000U)
#define BOARD_OSC_STARTUP_US      15625

#define PIN_PA0    IOPORT_CREATE_PIN(PIOA, 0)
#define PIN_PA1    IOPORT_CREATE_PIN(PIOA, 1)
#define PIN_PA2    IOPORT_CREATE_PIN(PIOA, 2)
#define PIN_PA3    IOPORT_CREATE_PIN(PIOA, 3)
#define PIN_PA4     IOPORT_CREATE_PIN(PIOA, 4)
#define PIN_PA5     IOPORT_CREATE_PIN(PIOA, 5)
#define PIN_PA6     IOPORT_CREATE_PIN(PIOA, 6)
#define PIN_PA7     IOPORT_CREATE_PIN(PIOA, 7)
#define PIN_PA8     IOPORT_CREATE_PIN(PIOA, 8)
#define PIN_PA11    IOPORT_CREATE_PIN(PIOA, 11)
#define PIN_PA12    IOPORT_CREATE_PIN(PIOA, 12)
#define PIN_PA13    IOPORT_CREATE_PIN(PIOA, 13)
#define PIN_PA14    IOPORT_CREATE_PIN(PIOA, 14)
#define PIN_PA15    IOPORT_CREATE_PIN(PIOA, 15)
#define PIN_PA16    IOPORT_CREATE_PIN(PIOA, 16)
#define PIN_PA17    IOPORT_CREATE_PIN(PIOA, 17)
#define PIN_PA18    IOPORT_CREATE_PIN(PIOA, 18)
#define PIN_PA19    IOPORT_CREATE_PIN(PIOA, 19)
#define PIN_PA20    IOPORT_CREATE_PIN(PIOA, 20)
#define PIN_PA21    IOPORT_CREATE_PIN(PIOA, 21)
#define PIN_PA22    IOPORT_CREATE_PIN(PIOA, 22)
#define PIN_PA23    IOPORT_CREATE_PIN(PIOA, 23)
#define PIN_PA24    IOPORT_CREATE_PIN(PIOA, 24)
#define PIN_PA25    IOPORT_CREATE_PIN(PIOA, 25)
#define PIN_PA27    IOPORT_CREATE_PIN(PIOA, 27)
#define PIN_PB0     IOPORT_CREATE_PIN(PIOB, 0)
#define PIN_PB1     IOPORT_CREATE_PIN(PIOB, 1)
#define PIN_PB2     IOPORT_CREATE_PIN(PIOB, 2)
#define PIN_PB3     IOPORT_CREATE_PIN(PIOB, 3)
#define PIN_PB4     IOPORT_CREATE_PIN(PIOB, 4)
#define PIN_PB5     IOPORT_CREATE_PIN(PIOB, 5)
#define PIN_PB10     IOPORT_CREATE_PIN(PIOB, 10)
#define PIN_PB11     IOPORT_CREATE_PIN(PIOB, 11)
#define PIN_PB12     IOPORT_CREATE_PIN(PIOB, 12)
#define PIN_PB13     IOPORT_CREATE_PIN(PIOB, 13)
#define PIN_PB14     IOPORT_CREATE_PIN(PIOB, 14)

// J8 ADC Bus connector pins
#define PIN_J8_7     PIN_PB10
#define PIN_J8_8     PIN_PA22
#define PIN_J8_9     PIN_PB11
#define PIN_J8_10    PIN_PB5
#define PIN_J8_11    PIN_PB13
#define PIN_J8_12    PIN_PB4
#define PIN_J8_13    PIN_PA1
#define PIN_J8_14    PIN_PA0
#define PIN_J8_15    PIN_PA16
#define PIN_J8_16    PIN_PA19
#define PIN_J8_17    PIN_PA15
#define PIN_J8_18    PIN_PA20
#define PIN_J8_19    PIN_PA17
#define PIN_J8_20    PIN_PA18
#define PIN_J8_21    PIN_PA21
#define PIN_J8_22    PIN_PA23

// Power control pins
#define PIN_ENABLE_PI_5V    PIN_PA5

// ADC control pins
#define PIN_ENABLE_ADC_PWR  PIN_PB10
#define PIN_ADC_SEL1		PIN_PB13
#define PIN_ADC_SEL0		PIN_PB11
#define PIN_ADC_SYNC		PIN_PA23

// PIN to enable SD Cards
#define PIN_SELECT_SD   PIN_PA6
#define PIN_ENABLE_SD1   PIN_PB1
#define PIN_ENABLE_SD2   PIN_PB0
#define SD_ENABLE   0
#define SD_DISABLE  1
#define SELECT_SD1   1
#define SELECT_SD2   0

// ADC preamp control pins
#define PIN_PREAMP_G0 PIN_PB4
#define PIN_PREAMP_G1 PIN_PB5
#define PIN_PREAMP_SHDN PIN_PA22

#define PIN_RTC_INT PIN_PA2

// LED definitions
//#define LED1_PIN      PIN_PB10 //PIN_PA7 //PIN_PA22
//#define LED2_PIN      PIN_PB11 //PIN_PA8 //PIN_PA7

#define LED_ON        true
#define LED_OFF       !LED_ON

//! Number of on-board LEDs
#define LED_COUNT 0

//! Number of on-board buttons
#define BUTTON_COUNT 0

//! Number of on-board UARTs
#define UART_COUNT 2

/** UART0 pins (UTXD0 and URXD0) definitions, PA9,10. */
#define PINS_UART0         (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART0_MASK    PIO_PA9A_URXD0|PIO_PA10A_UTXD0
#define PINS_UART0_PIO     PIOA
#define PINS_UART0_ID      ID_PIOA
#define PINS_UART0_TYPE    PIO_PERIPH_A
#define PINS_UART0_ATTR    PIO_DEFAULT

/** UART1 pins (UTXD1 and URXD1) definitions, PB2,PB3. */
#define PINS_UART1         (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART1_MASK    PIO_PB2A_URXD1 | PIO_PB3A_UTXD1
#define PINS_UART1_PIO     PIOB
#define PINS_UART1_ID      ID_PIOB
#define PINS_UART1_TYPE    PIO_PERIPH_A
#define PINS_UART1_ATTR    PIO_DEFAULT


//! \name HSMCI pins definition
/*! Number of slot connected on HSMCI interface */
#define SD_MMC_HSMCI_MEM_CNT      1
#define SD_MMC_HSMCI_SLOT_0_SIZE  4
//#define SD_MMC_HSMCI_SLOT_1_SIZE  4
#define PINS_HSMCI   {0x3fUL << 26, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_PULLUP}
/** HSMCI MCCDA pin definition. */
#define PIN_HSMCI_MCCDA_GPIO            (PIO_PA28_IDX)
//#define PIN_HSMCI_MCCDA_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
#define PIN_HSMCI_MCCDA_FLAGS           (PIO_PERIPH_C | PIO_PULLUP)
/** HSMCI MCCK pin definition. */
#define PIN_HSMCI_MCCK_GPIO             (PIO_PA29_IDX)
#define PIN_HSMCI_MCCK_FLAGS            (PIO_PERIPH_C | PIO_DEFAULT)
//#define PIN_HSMCI_MCCK_FLAGS            (PIO_PERIPH_C | PIO_PULLUP)
/** HSMCI MCDA0 pin definition. */
#define PIN_HSMCI_MCDA0_GPIO            (PIO_PA30_IDX)
//#define PIN_HSMCI_MCDA0_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
#define PIN_HSMCI_MCDA0_FLAGS           (PIO_PERIPH_C | PIO_PULLUP) 
/** HSMCI MCDA1 pin definition. */
#define PIN_HSMCI_MCDA1_GPIO            (PIO_PA31_IDX)
//#define PIN_HSMCI_MCDA1_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
#define PIN_HSMCI_MCDA1_FLAGS           (PIO_PERIPH_C | PIO_PULLUP)
/** HSMCI MCDA2 pin definition. */
#define PIN_HSMCI_MCDA2_GPIO            (PIO_PA26_IDX)
//#define PIN_HSMCI_MCDA2_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
#define PIN_HSMCI_MCDA2_FLAGS           (PIO_PERIPH_C | PIO_PULLUP)
/** HSMCI MCDA3 pin definition. */
#define PIN_HSMCI_MCDA3_GPIO            (PIO_PA27_IDX)
//#define PIN_HSMCI_MCDA3_FLAGS           (PIO_PERIPH_C | PIO_DEFAULT)
#define PIN_HSMCI_MCDA3_FLAGS           (PIO_PERIPH_C | PIO_PULLUP)
/** SD/MMC card detect pin definition. */

/** SSC pin Receiver Data (RD) */
#define PIN_SSC_RD        (PIO_PA18_IDX)
#define PIN_SSC_RD_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
/** SSC pin Transmitter Clock (TK) */
#define PIN_SSC_RK        (PIO_PA19_IDX)
#define PIN_SSC_RK_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
/** SSC pin Transmitter FrameSync (TF) */
#define PIN_SSC_RF        (PIO_PA20_IDX)
#define PIN_SSC_RF_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)

/** SSC pin Transmitter Data (TD) */
#define PIN_SSC_TD        (PIO_PA17_IDX)
#define PIN_SSC_TD_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
/** SSC pin Transmitter Clock (TK) */
#define PIN_SSC_TK        (PIO_PA16_IDX)
#define PIN_SSC_TK_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
/** SSC pin Transmitter FrameSync (TF) */
#define PIN_SSC_TF        (PIO_PA15_IDX)
#define PIN_SSC_TF_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)


// Timer Counters Pins
#define PIN_TC0_TIOA0 (PIO_PA0_IDX)
#define PIN_TC0_TIOA0_MUX   (IOPORT_MODE_MUX_B)
#define PIN_TC0_TIOA0_FLAGS (PIO_PERIPH_B | PIO_DEFAULT)

#define PIN_TC0_TIOB0 (PIO_PA1_IDX)
#define PIN_TC0_TIOB0_MUX   (IOPORT_MODE_MUX_B)
//#define PIN_TC0_TIOB0_FLAGS (PIO_PERIPH_B | PIO_DEFAULT)
#define PIN_TC0_TIOB0_FLAGS (PIO_PERIPH_B | PIO_PULLUP)

/** TWI0 pins definition */
#define PIN_TWI0_DATA_GPIO   PIO_PA3_IDX
#define PIN_TWI0_DATA_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_TWI0_CLK_GPIO    PIO_PA4_IDX
#define PIN_TWI0_CLK_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

/** TWI1 pins definition */
#define PIN_TWI1_DATA_GPIO   PIO_PB4_IDX
#define PIN_TWI1_DATA_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_TWI1_CLK_GPIO    PIO_PB5_IDX
#define PIN_TWI1_CLK_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

// SPI
/** SPI MISO pin definition. */
#define SPI_MISO_GPIO         (PIO_PA12_IDX)
#define SPI_MISO_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)
/** SPI MOSI pin definition. */
#define SPI_MOSI_GPIO         (PIO_PA13_IDX)
#define SPI_MOSI_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)
/** SPI SPCK pin definition. */
#define SPI_SPCK_GPIO         (PIO_PA14_IDX)
#define SPI_SPCK_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)

/** SPI chip select 0 pin definition. (Only one configuration is possible) */
//#define SPI_NPCS0_GPIO         (PIO_PA11_IDX)
//#define SPI_NPCS0_FLAGS           (PIO_PERIPH_A | PIO_DEFAULT)


#endif /* WISPR */
