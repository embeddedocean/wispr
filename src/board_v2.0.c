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

void board_init(void)
{
	
  pmc_disable_all_periph_clk();
  pmc_disable_udpck();

  pmc_enable_periph_clk(ID_PIOA);
  pmc_enable_periph_clk(ID_PIOB);

  // Initialize the user gpio pins
  board_gpio_init();
  
  // Initialize the console uart
  board_console_uart_init();
  
  // Use the RTC 32kHz output as the 32kHz xtal input
  int count = 100;
  //pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);  // enable external 32.768kHz crystal
  pmc_switch_sclk_to_32kxtal(PMC_OSC_BYPASS);  // enable external 32.768kHz clock - such as the DS3231 32khz output
  while (count--) {
    if(pmc_osc_is_ready_32kxtal()) break;  // wait until oscillator is ready
  }
  if(count == 0) printf("RTC 32KHz Clock not ready\r\n");

  /* Output header string  */
  fprintf(stdout, "\r\n-- %s --\r\n", BOARD_NAME );
  fprintf(stdout, "-- Compiled: "__DATE__ " "__TIME__ " --\r\n\r\n");

  //display_reset_reason();

  // Initialize the WDT
  board_wdt_init(0);

}

void board_gpio_init(void)
{
  ioport_init();

  //Set PB10 and PB11 as GPIO, not USB pins
  REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO10;
  REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO11;

  /* Configure SSC pins */
  pio_configure_pin(PIN_SSC_RD, PIN_SSC_RD_FLAGS);
  pio_configure_pin(PIN_SSC_RF, PIN_SSC_RF_FLAGS);
  pio_configure_pin(PIN_SSC_RK, PIN_SSC_RK_FLAGS);
  pio_configure_pin(PIN_SSC_TD, PIN_SSC_TD_FLAGS);
  pio_configure_pin(PIN_SSC_TF, PIN_SSC_TF_FLAGS);
  pio_configure_pin(PIN_SSC_TK, PIN_SSC_TK_FLAGS);
  
  /* Configure HSMCI pins */
  pio_configure_pin(PIN_HSMCI_MCCDA_GPIO, PIN_HSMCI_MCCDA_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCCK_GPIO, PIN_HSMCI_MCCK_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCDA0_GPIO, PIN_HSMCI_MCDA0_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCDA1_GPIO, PIN_HSMCI_MCDA1_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCDA2_GPIO, PIN_HSMCI_MCDA2_FLAGS);
  pio_configure_pin(PIN_HSMCI_MCDA3_GPIO, PIN_HSMCI_MCDA3_FLAGS);

  // Power control pins
  ioport_set_pin_dir(PIN_ENABLE_PI_5V, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_PI_5V, 0); // low is off

  // ADC control pins
  ioport_set_pin_dir(PIN_ENABLE_ADC_PWR, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 0); // low is off
  ioport_set_pin_dir(PIN_ADC_SYNC, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ADC_SYNC, 0);
  ioport_set_pin_dir(PIN_ADC_SEL0, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ADC_SEL0, 0);
  ioport_set_pin_dir(PIN_ADC_SEL1, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ADC_SEL1, 0);

  // SD Card pins
  ioport_set_pin_dir(PIN_SELECT_SD, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD1);

  ioport_set_pin_dir(PIN_ENABLE_SD1, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_SD1, SD_DISABLE);

  ioport_set_pin_dir(PIN_ENABLE_SD2, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_ENABLE_SD2, SD_DISABLE);

  // ADC Preamp control pins
  ioport_set_pin_dir(PIN_PREAMP_SHDN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_PREAMP_SHDN, 0);
  
  ioport_set_pin_dir(PIN_PREAMP_G0, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_PREAMP_G0, 0);
  
  ioport_set_pin_dir(PIN_PREAMP_G1, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(PIN_PREAMP_G1, 0);

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
	uart_opts.ul_mck = sysclk_get_peripheral_hz();
	uart_opts.ul_baudrate = baud;
	uart_opts.ul_mode = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;

	/* Configure console UART. */
	if(port == 0) { 
		sysclk_enable_peripheral_clock(ID_UART0);
		pio_configure_pin_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
		uart_init(UART0, &uart_opts);
	} else if(port == 1) {
		sysclk_enable_peripheral_clock(ID_UART1);
		pio_configure_pin_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);
		uart_init(UART1, &uart_opts);
	} else {
		stat = ERR_UNSUPPORTED_DEV;
	}

	return(stat);
}

uint32_t board_console_uart_init(void)
{
  uint32_t stat = 1;
	
  const usart_serial_options_t uart_serial_options = {
    .baudrate = BOARD_CONSOLE_UART_BAUDRATE,
    .paritytype = UART_MR_PAR_NO
  };

  /* Configure console UART. */
  if(BOARD_CONSOLE_UART == UART0) {
    sysclk_enable_peripheral_clock(ID_UART0);
    pio_configure_pin_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
    stdio_serial_init(UART0, &uart_serial_options);
  } else if(BOARD_CONSOLE_UART == UART1) {
    sysclk_enable_peripheral_clock(ID_UART1);
    pio_configure_pin_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);
    stdio_serial_init(UART1, &uart_serial_options);
  } else {
    stat = -1;
  }  
  return(stat);
}

//
// initialize WDT to the max timeout value (~15 seconds)
//
uint32_t board_wdt_init(uint32_t wdt_msec)
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
	printf("Enable watchdog with %d msec period (status %x)\n\r", msecs, wdt_get_status(WDT));

	return(msecs);
}

/**
 * \brief Display the reset reason(s).
 */
 void display_reset_reason(void)
 {
	char msg[80];
	
	strcpy(&msg[0], "\r\nReset info : ");
	
	//! [reset_get_status]
	uint32_t info = rstc_get_status(RSTC);
	//! [reset_get_status]
	
	/* Decode the reset reason. */
	switch (info & RSTC_SR_RSTTYP_Msk) {	
	case RSTC_GENERAL_RESET:
		strcat(&msg[0], "General Reset,");
	break;
	
	case RSTC_BACKUP_RESET:
		strcat(&msg[0], "Backup Reset,");
	break;
	
	case RSTC_WATCHDOG_RESET:
		strcat(&msg[0], "Watchdog Reset,");
	break;
	
	case RSTC_SOFTWARE_RESET:
		strcat(&msg[0], "Software Reset,");
	break;
	
	case RSTC_USER_RESET:
		strcat(&msg[0], "User Reset,");
	break;
	
	default:
		strcat(&msg[0], "Invalid reset reason!,");
	}
	
	/* NRST level. */
	if (info & RSTC_SR_NRSTL) {
		strcat(&msg[0], " NRST=1,");
	}
	else {
		strcat(&msg[0], " NRST=0,");
	}

	/* User reset status. */
	if (info & RSTC_SR_URSTS) {
		strcat(&msg[0], " User Reset=1\r");
	}
	else {
		strcat(&msg[0], " User Reset=0\r");
	}

	puts(&msg[0]);
 }
 