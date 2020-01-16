/*
 * rtc_time.h
 *
 * Created: 5/26/2017 9:39:16 AM
 *  Author: chris
 */ 

#ifndef _RTC_TIME_H_
#define _RTC_TIME_H_

#include <stdint.h>

typedef struct {
	uint8_t century;  // always 20
	uint8_t year;    // 0 - 99
	uint8_t month;   // 1 - 12
	uint8_t day;     // 1 - 31
	uint8_t hour;    // 0 - 23
	uint8_t minute;  // 0 - 59
	uint8_t second; // 0 - 59
	int8_t  tzoff;  // time zone offset from utc (hours)
} rtc_time_t;

extern void rtc_init( rtc_time_t *t );
extern void rtc_get_datetime( rtc_time_t *dt );
extern void rtc_get_epoch( uint32_t *epoch );


#endif /* RTC_TIME_H_ */