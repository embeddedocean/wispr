/**
 * \file
 *
 * Copyright (C) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <board.h>
#include <gpio.h>
#include <ioport.h>
#include <string.h>
#include <stdio.h>

#include <asf.h>
#include "board_v2.1.h"
#include "console.h"

/** Current MCK in Hz */
uint32_t board_current_cpu_hz;
uint32_t board_current_main_hz;

char reset_message[96];

int board_init(void)
{
  // initialize PLL and main clock with the settings in confog_clock.h 
  sysclk_init();
  
  //!< External 32kHz bypass oscillator as master source clock
  
  board_current_cpu_hz = sysclk_get_cpu_hz(); 
  board_current_main_hz = sysclk_get_main_hz();

  //board_init_clocks(mck);
  // need to figure out how to get sysclk_get_cpu_hz() to return a variable value
  // now it relies on constants in conf_clock.h
  	
  // Use the RTC 32kHz output as the 32kHz xtal input
//  pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);  // enable external 32.768kHz crystal
  pmc_switch_sclk_to_32kxtal(PMC_OSC_BYPASS);  // enable external 32.768kHz clock - such as the DS3231 32khz output
//  while (!pmc_osc_is_ready_32kxtal()) { }

  //ioport_init();
  pmc_enable_periph_clk(ID_PIOA);
  pmc_enable_periph_clk(ID_PIOB);

  // Configure all PIOs as inputs to save power
//  pio_set_input(PIOA, 0xFFFFFFFF, PIO_PULLUP);
//  pio_set_input(PIOB, 0xFFFFFFFF, PIO_PULLUP);

  // Initialize the console uart first so printf works
  console_init(BOARD_CONSOLE_PORT, BOARD_CONSOLE_BAUDRATE);

  // Initialize the user gpio pins
  board_gpio_init();

  // Initialize the 1Hz timer
  //board_init_timer();
    
  /* Output header string  */
  fprintf(stdout, "\r\n-- %s --\r\n", BOARD_NAME );
  fprintf(stdout, "-- Compiled: "__DATE__ " "__TIME__ " --\r\n");

  fprintf(stdout, "-- CPU clock: %lu MHz\n\r", board_current_cpu_hz/1000000);
  //fprintf(stdout, "-- Main clock: %lu MHz\n\r", board_current_main_hz/1000000);
  printf("-- Peripheral bus clock: %lu MHz\n\r",  sysclk_get_peripheral_hz()/1000000); //sysclk_get_peripheral_bus_hz(TC0)/1000000);

  // get the reset reasons
  uint8_t reset_type = 0;
  uint8_t nrst, user_reset;
  board_reset_reason(&reset_type, &nrst, &user_reset);
  fprintf(stdout, "-- %s\r\n", reset_message );
  fprintf(stdout, "\r\n" );
  
  return(reset_type);
}

// not finished 
uint32_t board_get_cpu_clock_hz(void)
{
	//= board_current_main_hz / ((pll_pre == SYSCLK_PRES_3) ? 3 : (1 << (pll_pre >> PMC_MCKR_PRES_Pos)));
	return(board_current_cpu_hz);
}

uint32_t board_get_main_clock_hz(void)
{
	return(board_current_main_hz);
}

void board_set_clock(enum board_cpu_freq_hz cpu_hz)
{
	// redefines the pll multiplier
	// everything else in conf_clock.h stays the same
	// 
	
	uint32_t osc_hz = osc_get_rate(CONFIG_PLL0_SOURCE);
	uint32_t pll_div = CONFIG_PLL0_DIV;
	uint32_t pll_mul = cpu_hz / osc_hz; 
	
	board_current_cpu_hz = osc_hz * pll_mul / pll_div;

	pmc_set_writeprotect(0);

	struct pll_config pllcfg;
	pll_enable_source(CONFIG_PLL0_SOURCE);
	pll_config_init(&pllcfg, CONFIG_PLL0_SOURCE, pll_div, pll_mul);
	pll_enable(&pllcfg, 0);
	pll_wait_for_lock(0);
	pmc_switch_mck_to_pllack(CONFIG_SYSCLK_PRES);

	/* Update the SystemFrequency variable */
	SystemCoreClockUpdate();

	/* Set a flash wait state depending on the new cpu frequency */
	system_init_flash(sysclk_get_cpu_hz());

	//pll_enable_source(OSC_MAINCK_BYPASS);
	//pmc_switch_mainck_to_xtal(PMC_OSC_BYPASS, pmc_us_to_moscxtst(BOARD_OSC_STARTUP_US, OSC_SLCK_32K_RC_HZ));
	//while (!pmc_osc_is_ready_mainck()) {}
	
	// enable PLLA (need to use pll_mul-1)
//	pmc_enable_pllack(pll_mul, PLL_COUNT, pll_div);

	// Switch master clock source selection to PLLA clock
//	pmc_switch_mck_to_pllack(pll_pre);

	/* Disable unused clock to save power */
//	pmc_osc_disable_fastrc();
//	pmc_disable_all_periph_clk();
//	pmc_disable_udpck();

	// enable/disable peripheral clocks as they are needed

}

static void rtc_wakeup_handler(uint32_t ul_id, uint32_t ul_mask)
{
	if (ID_PIOA == ul_id && PIO_PA2 == ul_mask) {
		//printf("RTC Interrupt Handler\r\n");
	}
}

void board_init_wakeup(void)
{
  // setup RTC_INT_PIN used for wakeup on PA2 (WKUP2)
  ioport_set_pin_dir(PIN_RTC_INT, IOPORT_DIR_INPUT);
  ioport_set_pin_mode(PIN_RTC_INT, IOPORT_MODE_PULLUP);
  
  // Initialize PIO interrupt handler for PA2 (WKUP2) with falling edge or low detection
  uint32_t attr = (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_LOW_LEVEL);
  pio_handler_set(PIN_RTC_INT_PIO, PIN_RTC_INT_ID, PIN_RTC_INT_MASK, attr, rtc_wakeup_handler);

  //pio_handler_set(PIOA, ID_PIOA, PIO_PA2, (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_LOW_LEVEL), rtc_wakeup_handler);
 
  // Enable PIOA controller IRQs
  //NVIC_DisableIRQ(PIOA_IRQn);
  //NVIC_ClearPendingIRQ(PIOA_IRQn);
  //NVIC_SetPriority(PIOA_IRQn, 0);
  NVIC_EnableIRQ(PIOA_IRQn);

  // Enable PIOA line interrupts
  pio_enable_interrupt(PIN_RTC_INT_PIO, PIN_RTC_INT_MASK);

  uint32_t inputs = 0;
  uint32_t transitions = 0;

  // wakeup input on WKUP2 (PA2) is the external rtc interrupt line
  inputs |= SUPC_WUIR_WKUPEN2_ENABLE;
  transitions |= SUPC_WUIR_WKUPT2_LOW; // wakeup on input transition to low

  // wakeup input on WKUP12 (PB2) is the UART1 RX pin
  inputs |= SUPC_WUIR_WKUPEN12_ENABLE;
  transitions |= SUPC_WUIR_WKUPT12_LOW; // wakeup on input transition to low

  // wakeup input on WKUP6 (PA9) is the UART0 RX pin
  inputs |= SUPC_WUIR_WKUPEN6_ENABLE;
  transitions |= SUPC_WUIR_WKUPT6_LOW; // wakeup on input transition to low

  // wakeup mode - set debounce counter value 
  uint32_t mode = SUPC->SUPC_WUMR;
  //mode |= SUPC_WUMR_WKUPDBC_3_SCLK; // WKUPx shall be in its active state for at least 3 SLCK periods
  mode |= SUPC_WUMR_WKUPDBC_IMMEDIATE; // no debouncing, active state at least on one Slow Clock edge
  
  // enable wakeup input on WKUP2 (PA2) with active low
  // this is the external rtc interrupt line
  supc_set_wakeup_inputs(SUPC, inputs, transitions);
  supc_set_wakeup_mode(SUPC, mode);

}


void board_gpio_init(void)
{

  //Set PB10 and PB11 as GPIO, not USB pins
  REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO10;
  REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO11;

  // debug pins - when not used for USB
  ioport_set_pin_dir(PIN_PB10, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_PB10, 0);
  ioport_set_pin_dir(PIN_PB11, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_PB11, 0);

  // Power control pins
  ioport_set_pin_dir(PIN_ENABLE_5V, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_5V, 0); // low is off

  ioport_set_pin_dir(PIN_ENABLE_5V5, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_5V5, 0); // low is off

  // ADC control pins
  ioport_set_pin_dir(PIN_ENABLE_ADC_PWR, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 0); // low is off

  ioport_set_pin_dir(PIN_ADC_SYNC, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ADC_SYNC, 0);

  ioport_set_pin_dir(PIN_ADC_SEL0, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ADC_SEL0, 0);

  ioport_set_pin_dir(PIN_ADC_SEL1, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ADC_SEL1, 0);

  // ADC Preamp control pins
  ioport_set_pin_dir(PIN_PREAMP_SHDN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_PREAMP_SHDN, 0);
  
  ioport_set_pin_dir(PIN_PREAMP_G0, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_PREAMP_G0, 0);
  
  ioport_set_pin_dir(PIN_PREAMP_G1, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_PREAMP_G1, 0);

  // SD Card pins
  /* mnoved to driver
  ioport_set_pin_dir(PIN_SELECT_SD, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD1);

  ioport_set_pin_dir(PIN_ENABLE_SD1, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_SD1, SD_DISABLE);

  ioport_set_pin_dir(PIN_ENABLE_SD2, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_SD2, SD_DISABLE);

  // Configure HSMCI pins 
  pio_configure_pin(PIN_HSMCI_MCCDA_GPIO, PIN_HSMCI_MCCDA_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCCK_GPIO, PIN_HSMCI_MCCK_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCDA0_GPIO, PIN_HSMCI_MCDA0_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCDA1_GPIO, PIN_HSMCI_MCDA1_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCDA2_GPIO, PIN_HSMCI_MCDA2_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCDA3_GPIO, PIN_HSMCI_MCDA3_FLAGS);
  */

  /* Configure SSC pins  - moved to driver
  pio_configure_pin(PIN_SSC_RD, PIN_SSC_RD_FLAGS);
  pio_configure_pin(PIN_SSC_RF, PIN_SSC_RF_FLAGS);
  pio_configure_pin(PIN_SSC_RK, PIN_SSC_RK_FLAGS);
  pio_configure_pin(PIN_SSC_TD, PIN_SSC_TD_FLAGS);
//  pio_configure_pin(PIN_SSC_TF, PIN_SSC_TF_FLAGS);
  pio_configure_pin(PIN_SSC_TK, PIN_SSC_TK_FLAGS);
  */ 
    
}

// 
// UART init
//
uint32_t board_uart_init(int port, uint32_t baud)
{
	enum status_code stat = STATUS_OK;
	if( (port < 0) || (port >= UART_COUNT) ) {
		return(ERR_INVALID_ARG);
	}

	sam_uart_opt_t uart_opts;
	//uart_opts.ul_mck = sysclk_get_peripheral_hz(); // relies on conf_clock.h;
	uart_opts.ul_mck = board_get_cpu_clock_hz(); // use variable rate in case it has changed
	uart_opts.ul_baudrate = baud;
	uart_opts.ul_mode = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;

	/* Configure UART. */
	if(port == 0) { 
		//pmc_disable_periph_clk(ID_UART0);
		pmc_enable_periph_clk(ID_UART0);
		pio_configure_pin_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
		uart_init(UART0, &uart_opts);
	} else if(port == 1) {
		pmc_enable_periph_clk(ID_UART1);
		pio_configure_pin_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);
		uart_init(UART1, &uart_opts);
	} else {
		stat = ERR_UNSUPPORTED_DEV;
	}

	return(stat);
}

//
// console is where printf goes (using usart_serial_putchar and usart_serial_getchar)
//
/* This is now defined in console.c
uint32_t board_console_init(int port, uint32_t baud)
{
  uint32_t stat = 1;
	
  const usart_serial_options_t uart_serial_options = {
    .baudrate = baud,
    .paritytype = UART_MR_PAR_NO
  };

  // Configure console UART.
  if(port == 0) {
	board_uart_init(0, baud);
    stdio_serial_init(UART0, &uart_serial_options);
  } else if(port == 1) {
	board_uart_init(1, baud);
    stdio_serial_init(UART1, &uart_serial_options);
  } else {
    stat = -1;
  }  
  return(stat);
}
*/
  
//
// Initialize WDT to the max timeout value (~15 seconds)
// WDT_MR can be written only once. Only a processor reset resets it. 
// Writing WDT_MR reloads the timer with the newly programmed mode parameters.
//
uint32_t board_wdt_init(void)
{
	uint32_t timeout_value = 4095; //max timeout value;
	//uint32_t timeout_value = wdt_get_timeout_value(1000*wdt_msec, BOARD_FREQ_SLCK_XTAL);
	uint32_t msecs = 0;

	/* Configure WDT to trigger a reset */
	uint32_t wdt_mode = WDT_MR_WDRSTEN | WDT_MR_WDRPROC;  /* WDT fault resets processor only. */
	//WDT_MR_WDDBGHLT  |  /* WDT stops in debug state. */
	//WDT_MR_WDIDLEHLT;   /* WDT stops in idle state. */

	/* Initialize WDT with the given parameters. */
	wdt_init(WDT, wdt_mode, timeout_value, timeout_value);
	
	msecs = wdt_get_us_timeout_period(WDT, BOARD_FREQ_SLCK_XTAL) / 1000;

	//printf("Enable watchdog with %d msec period (status %x)\n\r", msecs, wdt_get_status(WDT));

	return(msecs);
}

/**
 * \brief Display the reset reason(s).
 */
 void board_reset_reason(uint8_t *reason, uint8_t *nrst, uint8_t *user)
{
	char *msg = &reset_message[0];
	
	// get reset status
	uint32_t info = rstc_get_status(RSTC);
		
	// Decode the reset reason.
	switch (info & RSTC_SR_RSTTYP_Msk) {	
	
	case RSTC_GENERAL_RESET:
		strcat(&msg[0], "General Reset,");
		*reason = BOARD_GENERAL_RESET;
	break;
	
	case RSTC_BACKUP_RESET:
		*reason = BOARD_BACKUP_RESET;
		strcat(&msg[0], "Backup Reset,");
	break;
	
	case RSTC_WATCHDOG_RESET:
		*reason = BOARD_WATCHDOG_RESET;
		strcat(&msg[0], "Watchdog Reset,");
	break;
	
	case RSTC_SOFTWARE_RESET:
		*reason = BOARD_SOFTWARE_RESET;
		strcat(&msg[0], "Software Reset,");
	break;
	
	case RSTC_USER_RESET:
		*reason = BOARD_USER_RESET;
		strcat(&msg[0], "User Reset,");
	break;
	
	default:
		*reason = BOARD_UNKNOWN_RESET;
		strcat(&msg[0], "Invalid reset reason,");
	}
	
	/* NRST level. */
	if (info & RSTC_SR_NRSTL) {
		*nrst = 1;
		strcat(&msg[0], " NRST=1,");		
	}
	else {
		*nrst = 0;
		strcat(&msg[0], " NRST=0,");
	}

	/* User reset status. */
	if (info & RSTC_SR_URSTS) {
		*user = 1;
		strcat(&msg[0], " User Reset=1");
	}
	else {
		*user = 0;
		strcat(&msg[0], " User Reset=0");
	}

	//return( &msg[0] ); 
 }
 