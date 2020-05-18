/*
 * pps_timer.c
 *
 * Created: 5/8/2020 2:09:22 AM
 *  Author: Chris
 */ 

#include <asf.h>
#include <stdio.h>
#include <stdint.h>

#include "board_v2.0.h"
#include "rtc_time.h"
#include "pps_timer.h"

#include "ltc2512.h"

uint32_t pps_timer_start_second = 0; // the time pps timer was initialized (unix time)

volatile uint8_t pps_timer_sync_flag = 0;

//
// RTT Alarm interrupt routine updates the start second when the counter rolls over to zero.
// This shouldn't happen very often. 
// With prescaler=4 this alarm will occur 32768 seconds after it starts.
//
void RTT_Handler(void)
{	
	uint32_t status = rtt_get_status(RTT);

	// counter has rolled over
	if ( status & RTT_SR_ALMS ) {

		// reset the start time seconds
		uint32_t start;
		rtc_get_epoch(&start);
		pps_timer_start_second = start;
		
		printf("rtt alarm interrupt: new start time %d\r\n", pps_timer_start_second);
	}
}

//
// PPS Timer Setup
//
uint32_t pps_timer_init(void)
{
	sysclk_enable_peripheral_clock(ID_RTT);
	rtt_disable(RTT);
	
	// get the start time epoch
	uint32_t status = rtc_get_epoch(&pps_timer_start_second);
	
	// Reset the rtt - sets the prescaler and reset bits, clears all other bits in MR.
	// So by default the rtt clock source is 32k slow clock, not interrupts, ...
	// This also resets the 32-bit counter.
	rtt_init(RTT, PPS_RTT_PRESCALER);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN); // enable the alarm interrupt
	rtt_write_alarm_time(RTT, 0xFFFFFFFF); // set alarm for counter roll over to zero
	rtt_enable(RTT);
	
	// setup the pin used for pp input
	ioport_set_pin_dir(PIN_PPS_INT, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(PIN_PPS_INT, IOPORT_MODE_PULLDOWN|IOPORT_MODE_GLITCH_FILTER);

	// Enable RTT controller IRQs
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, PPS_IRQ_PRIO+1);
	NVIC_EnableIRQ(RTT_IRQn);
	
	return(status);
}

void pps_timer_stop(void)
{
	// Disable PIOA line interrupts
	NVIC_DisableIRQ((IRQn_Type)ID_PIOA);
	//pio_disable_interrupt(PIOA, PIO_PA2);
	pio_disable_interrupt(PIN_PPS_PIO, PIN_PPS_MASK);

	// stop the rtt
	rtt_disable(RTT);
}

void pps_timer_read(uint32_t *sec, uint32_t *usec)
{
	uint32_t count = rtt_read_timer_value(RTT);
	float fsec = (float)(PPS_RTT_PRESCALER * count) / 32768.0f;
	*sec = (uint32_t)fsec;
	*usec = (uint32_t)((fsec - (float)(*sec))*1000000.0);
	*sec = pps_timer_start_second + (uint32_t)fsec;
}


//
// PPS Sync Callback Interrupt Handlers
//

// pointer to the callback function 
uint32_t (*pps_timer_sync_callback)(void);
uint32_t pps_timer_cb_status = 0;
//
// interrupt routine that resets the rtt and executes the callback function
//
static void pps_sync_cb_handler(uint32_t id, uint32_t mask)
{
	if ( (ID_PIOA == id) && (PIO_PA2 == mask)) {
		
		if ( pps_timer_sync_flag ) {

			// Reset the rtt - sets the prescaler and reset bits, clears all other bits in MR.
			// So by default the rtt clock source is 32k slow clock, no interrupts, ...
			// This also resets the 32-bit counter.
			rtt_init(RTT, PPS_RTT_PRESCALER);
			rtt_enable_interrupt(RTT, RTT_MR_ALMIEN); // enable the alarm interrupt
			rtt_write_alarm_time(RTT, 0xFFFFFFFF); // set alarm for counter roll over to zero
			rtt_enable(RTT);

			// Disable PIOA line interrupts after the first sync interrupt
			NVIC_DisableIRQ((IRQn_Type)PIN_PPS_ID);
			pio_disable_interrupt(PIN_PPS_PIO, PIN_PPS_MASK);
			
			// Execute the callback function
			pps_timer_cb_status = pps_timer_sync_callback();

			// set the start time in case it changed during the callback
			uint32_t start;
			rtc_get_epoch(&start);
			pps_timer_start_second = start;
	
			pps_timer_sync_flag = 0;
		}	
	}
}

//
// Synchronize the pps with the specified callback function.
// The specified callback function is executed on the next pps rising edge.
// Waits for the the call back to be executed and returns the return value of the callback function. 
//
uint32_t pps_timer_sync( uint32_t (*func)(void) )
{
	// Enable PIOA controller IRQs
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, PPS_IRQ_PRIO);
	NVIC_EnableIRQ(PIOA_IRQn);

	// Initialize PIO interrupt handler for PPS with rising edge detection
	uint32_t attr = (PIO_DEBOUNCE | PIO_IT_RISE_EDGE);
    uint32_t pio_status = pio_handler_set(PIN_PPS_PIO, PIN_PPS_ID, PIN_PPS_MASK, attr, pps_sync_cb_handler);
	if(pio_status != 0) {
		printf("pps_timer_sync: pio_handle_set failed\r\n");
		return(pio_status);
	}
	
	// Enable PIOA line interrupts
	pio_enable_interrupt(PIN_PPS_PIO, PIN_PPS_MASK);

	// set the start time
	uint32_t start;
	rtc_get_epoch(&start);
	pps_timer_start_second = start;

	// set the callback function
	pps_timer_sync_callback = func;

	pps_timer_sync_flag = 1;

	// wait for call back interrupt
	// make sure this flag (pps_timer_sync_value) is volatile
	while( pps_timer_sync_flag > 0 ) {}
	
	//printf("pps_timer_sync: start time %d\r\n", pps_timer_start_second);
	
	return(pps_timer_cb_status);
}

//
// PPS calibration Interrupt Handler
//
volatile uint8_t pps_timer_cal_count = 0;
volatile uint8_t pps_timer_cal_start = 0;
uint32_t pps_timer_cal_start_value = 0;
uint32_t pps_timer_cal_stop_value = 0;
extern uint32_t board_current_cpu_hz;

static void pps_cal_handler(uint32_t id, uint32_t mask)
{
	if ( (ID_PIOA == id) && (PIO_PA2 == mask)) {
		
		// rtt calibration
		if ( pps_timer_cal_count ) {

			// if first interrupt, set start count value
			if(pps_timer_cal_count == pps_timer_cal_start) { 
				pps_timer_cal_start_value = rtt_read_timer_value(RTT);
			}

			// if last interrupt, set stop count value
			if(pps_timer_cal_count == 1) { 
				pps_timer_cal_stop_value = rtt_read_timer_value(RTT);
				// disable future interrupts
				NVIC_DisableIRQ((IRQn_Type)ID_PIOA);
				pio_disable_interrupt(PIOA, PIO_PA2);
			}
			
			// decrement the counter flag
			pps_timer_cal_count--;
			
			printf("pps_interrupt: calibration count %x\r\n", pps_timer_cal_count);

		}
	}
}

//
// Calibrate the system slow clock relative to the pps by measuring the number of rtt counts in a pps cycle.
// A minimum of 2 pps cycles is required with a maximum of 10.
// If the slow clock is driven by the cpu clock, this can be used to adjust the cpu clock.
//
uint32_t pps_timer_calib(void)
{
	uint8_t cycles = PS_TIMER_CAL_CYCLES;
	if( cycles < 2) cycles = 2;
	if( cycles > 10) cycles = 10;
	
	// Enable PIOA controller IRQs
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, PPS_IRQ_PRIO);
	NVIC_EnableIRQ(PIOA_IRQn);

	// Initialize PIO interrupt handler for PA2 with rising edge detection
	//uint32_t pio_status = pio_handler_set(PIOA, ID_PIOA, PIO_PA2, (PIO_DEBOUNCE | PIO_IT_RISE_EDGE), pps_cal_handler);
	uint32_t attr = (PIO_DEBOUNCE | PIO_IT_RISE_EDGE);
    uint32_t pio_status = pio_handler_set(PIN_PPS_PIO, PIN_PPS_ID, PIN_PPS_MASK, attr, pps_cal_handler);
	if(pio_status != 0) {
		printf("pps_timer_sync: pio_handle_set failed\r\n");
		return(pio_status);
	}
	
	// Enable PIOA line interrupts
	//pio_enable_interrupt(PIOA, PIO_PA2);
	pio_enable_interrupt(PIN_PPS_PIO, PIN_PPS_MASK);
	
	// set cal pps counter flag
	pps_timer_cal_start = cycles + 1;
	pps_timer_cal_count = cycles + 1;
	
	// wait for cal to finish
	// make sure this flag (pps_timer_cal_count) is volatile
	while(pps_timer_cal_count > 0) {}
	
	// check if something went wrong, like the counter rolled over during the cal
	if( pps_timer_cal_stop_value <= pps_timer_cal_start_value )	{
		printf("pps_timer_calib: failed\r\n");
		return(0);
	}
	
	uint32_t delta = pps_timer_cal_stop_value - pps_timer_cal_start_value;
	float cal = (float)cycles * 32768.0f / (float)delta / (float)PPS_RTT_PRESCALER;
	
	// Adjusting the cpu clock only makes sense if the slow clock and cpu are driven by the same source.
	// Recall that the rtt is driven by the slow clock.
	//uint32_t cpu_hz = (uint32_t)((float)board_current_cpu_hz  * cal);
	//board_current_cpu_hz = cpu_hz;
	
	// actual slow clock freq in hz
	uint32_t sclk = (uint32_t)(32768.0  * cal);
		
	printf("pps_timer_calib: actual slow clock %d hz\r\n", sclk);
	
	return(sclk);
}



