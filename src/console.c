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
Uart *CONSOLE_UART = 0; //BOARD_CONSOLE_UART;

int console_echo = 1;

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


//
// this is like a gets with a timeout 
// and it resets the WDT as it waits for input
// 
int console_input(char *str, int size, int timeout)
{
	uint8_t c;
	int n = 0;
	int go = 1;
	int timeout_count = 0;
	
	if( timeout <= 0 ) timeout = 60; // always have some timeout
	int timeout_ms = timeout*1000;

	str[0] = 0;
	
	while( go ) {

		if(!uart_is_rx_ready(CONSOLE_UART)) {
			if((timeout > 0) && (timeout_count >= timeout_ms)) {
				n = 0; // ignore chars that has been read 
				break;
			}
			wdt_restart(WDT);
			delay_ms(1);
			timeout_count++;
			continue;
		}

		uart_read(CONSOLE_UART, &c);

		// termination
		if((c == 13)||(c == 10)) { // CR or NL
			str[n] = 0;
			break;
		}

		// echo 
		if( console_echo && (c != 8) ) {
			while(!uart_is_tx_ready(CONSOLE_UART)) {}
			uart_write(CONSOLE_UART, c);  // echo
		}

		// backspace
		if( c == 8 && n > 0 ) {
			n = n-1;
			while(!uart_is_tx_ready(CONSOLE_UART)) {}
			uart_write(CONSOLE_UART, c);  // echo
			while(!uart_is_tx_ready(CONSOLE_UART)) {}
			uart_write(CONSOLE_UART, ' ');  // echo
			while(!uart_is_tx_ready(CONSOLE_UART)) {}
			uart_write(CONSOLE_UART, c);  // echo
		}
		
		if( c != 8 ) str[n++] = (char)c;

		if(n >= size) break;

	}	
	return(n);
}

int console_prompt_str(char *out, int nbytes, const char *prompt, char *default_value, int timeout)
{
	char in[32];
	fprintf(stdout, "\r\n%s [%s]: ", prompt, default_value);
	int n = console_input(in, nbytes, timeout);
	if(n > 0) strcpy(out, in);
	else strcpy(out, default_value);
	fprintf(stdout, "\r\n");
	return(strlen(out));
}

int console_prompt_int(const char *prompt, int default_value, int timeout)
{
	char str[32];
	fprintf(stdout, "\r\n%s [%d]: ", prompt, default_value);
	int n = console_input(str, 32, timeout);
	int value = default_value;
	if(n > 0) sscanf(str, "%d", &value); //value = atoi(str);
	//fprintf(stdout, "\r\n %d chars, str=%s, value=%d\r\n", n, str, value);
	fprintf(stdout, "\r\n");
	return(value);
}


float console_prompt_f32(const char *prompt, float default_value, int timeout)
{
	char str[64];
	fprintf(stdout, "\r\n%s [%f]: ", prompt, default_value);
	int n = console_input(str, 64, timeout);
	float value = default_value;
	if(n > 0) sscanf(str, "%f", &value); //value = atoff(str);
	//fprintf(stdout, "\r\n %d chars, str=%s, value = %f\r\n", n, str, value);
	fprintf(stdout, "\r\n");
	return(value);
}

uint32_t console_prompt_uint32(const char *prompt, uint32_t default_value, int timeout)
{
	return( (uint32_t)console_prompt_int(prompt, (int)default_value, timeout) );
}

uint16_t console_prompt_uint16(const char *prompt, uint16_t default_value, int timeout)
{
	return( (uint16_t)console_prompt_int(prompt, (int)default_value, timeout) );
}

uint8_t console_prompt_uint8(const char *prompt, uint8_t default_value, int timeout)
{
	return( (uint8_t)console_prompt_int(prompt, (int)default_value, timeout) );
}

//
// string parsing
//
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
			if(!strcmp("value1", tok)) {
				tok = strtok(NULL, delims);
				int value1 = atoi(tok);
				fprintf(stdout, "\r\n value1 = %d\r\n", value1);
			}
			// upload interval
			if(!strcmp("value2", tok)) {
				tok = strtok(NULL, delims);
				float value2 = atof(tok);
				fprintf(stdout, "\r\n value2 = %f\r\n", value2);
			}	
			tok = strtok(NULL, delims);
		}

	}
	
	return(1);
}

