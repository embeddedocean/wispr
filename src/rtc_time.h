/*
 * rtc_time.h
 *
 * Created: 5/26/2017 9:39:16 AM
 *  Author: chris
 */ 

#ifndef _RTC_TIME_H_
#define _RTC_TIME_H_

#include <stdint.h>

#define RTC_STATUS_OK 0
#define RTC_STATUS_NVTIM 1
#define RTC_STATUS_NVCAL 2

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

extern uint32_t rtc_init( rtc_time_t *t );
extern uint32_t rtc_get_datetime( rtc_time_t *dt );
extern uint32_t rtc_set_datetime( rtc_time_t *dt );
extern uint32_t rtc_get_epoch( uint32_t *epoch );
extern uint8_t rtc_valid_datetime(const rtc_time_t *dt);

extern void rtc_get_datetime_asf( rtc_time_t *dt );

#endif /* RTC_TIME_H_ */