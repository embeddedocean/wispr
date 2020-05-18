/*
 * pps_timer.h
 *
 * Created: 5/8/2020 2:09:43 AM
 *  Author: Chris
 */ 


#ifndef PPS_TIMER_H_
#define PPS_TIMER_H_

/* The PPS interrupt IRQ priority. */
#define PPS_IRQ_PRIO 0

// RTT prescaler defines the rtt resolution, 3 is the minimum value (1,2 are forbidden).
// With the 32K clock driving the rtt, the timer resolution is prescaler/32768 seconds per count
#define PPS_RTT_PRESCALER 4

// number of pps cycles to use for calibration
#define PS_TIMER_CAL_CYCLES 2

extern uint32_t pps_timer_init(void);
extern void pps_timer_stop(void);
extern void pps_timer_read(uint32_t *sec, uint32_t *usec);
extern uint32_t pps_timer_sync(uint32_t (*func)(void));
extern uint32_t pps_timer_calib(void);


#endif /* PPS_TIMER_H_ */