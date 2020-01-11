/*
 * console.c
 *
 * Created: 5/24/2017 
 *  Author: chris
 */ 

#include <asf.h>
#include <string.h>
#include <stdio.h>
#include "console.h"
#include "uart_queue.h"

#include "board.h"  
Uart *CONSOLE_UART = BOARD_CONSOLE_UART;

/**
 *  Configure UART console on UART0
 */
void console_init(int port, uint32_t baud)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = baud,
		.paritytype = UART_MR_PAR_NO
	};

	/* Configure console UART. */
	if(port == 0) {
		sysclk_enable_peripheral_clock(ID_UART0);
		pio_configure_pin_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
		stdio_serial_init(UART0, &uart_serial_options);
		CONSOLE_UART = UART0;
	}
	if(port == 1) {
		sysclk_enable_peripheral_clock(ID_UART1);
		pio_configure_pin_group(PINS_UART1_PIO, PINS_UART1, PINS_UART1_FLAGS);
		stdio_serial_init(UART1, &uart_serial_options);
		CONSOLE_UART = UART1;
	}
	
}

int console_gets(char *str)
{
	uint8_t c;
	int n = 0;
	int go = 1;
	while( go ) {
		if(uart_is_rx_ready(CONSOLE_UART)) {
			uart_read(CONSOLE_UART, &c);
			if(c == 13) { // newline
				str[n] = 0;
				go = 0;
			} else {
				uart_write(CONSOLE_UART, c);
				str[n] = (char)c;
			}
			if(c==8 && n>0) n--; // backspace
			n++;
		}
	}
	return(n);
}

int console_prompt_int(const char *prompt, int default_value, int timeout)
{
	uint8_t c;
	char str[64];
	int n = 0;
	int go = 1;
	int count = 0;
	
	if( timeout <= 0 ) timeout = 60; // always have some timeout
	int timeout_ms = timeout*1000;

	fprintf(stdout, "\r\n%s [%d]: ", prompt, default_value);

	while( go ) {
		if(uart_is_rx_ready(CONSOLE_UART)) {
			uart_read(CONSOLE_UART, &c);
			if(c == 13) { // newline
				str[n] = 0;
				go = 0;
			} else {
				uart_write(CONSOLE_UART, c);  // echo
				str[n] = (char)c;
			}
			if(c==8 && n>0) n--; // backspace
			n++;
		} else {
			delay_ms(1);
			count++;
		}

		// restart the wdt counter every second
		if( count >= 1000 ) wdt_restart(WDT);

		// timeout
		if((timeout > 0) && (count >= timeout_ms)) go = 0;

	}
	
	int value = default_value;
	if((n > 1) && (count < timeout_ms) ) value = atoi(str);
	
	//fprintf(stdout, "\r\n %d chars, %s, value = %d count = %d, %d \r\n", n, str, value, count, timeout);
	fprintf(stdout, "\r\n");

	return(value);
}

uint32_t console_prompt_uint32(const char *prompt, uint32_t default_value, int timeout)
{
	return((uint32_t)console_prompt_int(prompt, (int)default_value, timeout));
}

uint16_t console_prompt_uint16(const char *prompt, uint16_t default_value, int timeout)
{
	return((uint16_t)console_prompt_int(prompt, (int)default_value, timeout));
}

uint8_t console_prompt_uint8(const char *prompt, uint8_t default_value, int timeout)
{
	return((uint8_t)console_prompt_int(prompt, (int)default_value, timeout));
}

int read_console_input(void)
{
	char str[128]; 
	int go = 1;
	while( go ) {

		char delims[] = " =:,";
		char *tok;
		tok = strtok(str, delims);
		while(tok) {
			// sample rate		
			if(!strcmp("sampling_rate", tok)) {
				tok = strtok(NULL, delims);
				//sampling_rate = (uint32_t)atoi(tok);
				//fprintf(stdout, "\r\n set sample_rate = %d\r\n", sampling_rate);
			}
			// upload interval
			if(!strcmp("upload_interval_in_minutes", tok)) {
				tok = strtok(NULL, delims);
				//upload_interval_in_minutes = (uint32_t)atoi(tok);
				//fprintf(stdout, "\r\n set upload_interval_in_minutes = %d\r\n", upload_interval_in_minutes);
			}	
			tok = strtok(NULL, delims);
		}

	}
	
	return(1);
}

