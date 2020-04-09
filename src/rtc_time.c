/*
 * rtc_time.c
 *
 * Created: 5/26/2017 9:37:31 AM
 *  Author: chris
 */ 
#include <asf.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "rtc_time.h"
#include "epoch.h"

/**
 * Calculate week from year, month, day.
 */
static uint32_t calculate_week(uint32_t ul_year, uint32_t ul_month, uint32_t ul_day)
{
	uint32_t ul_week;

	if (ul_month == 1 || ul_month == 2) {
		ul_month += 12;
		--ul_year;
	}

	ul_week = (ul_day + 2 * ul_month + 3 * (ul_month + 1) / 5 + ul_year +
			ul_year / 4 - ul_year / 100 + ul_year / 400) % 7;

	++ul_week;

	return ul_week;
}

void rtc_init( rtc_time_t *dt )
{
	uint32_t year = (uint32_t)(dt->century*100 + dt->year);
	uint32_t month = (uint32_t)(dt->month);
	uint32_t day = (uint32_t)(dt->day);
	uint32_t hour = (uint32_t)(dt->hour);
	uint32_t minute = (uint32_t)(dt->minute);
	uint32_t sec = (uint32_t)(dt->second);
	
	uint32_t week = calculate_week(year, month, day);
	//uint32_t usec = 0;
	
	rtc_set_hour_mode(RTC, 0);
	rtc_set_time(RTC, hour, minute, sec);
	rtc_set_date(RTC, year, month, day, week);

	//printf("Internal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n", year, month, day, hour, minute, sec);
	
	// read time back to verify
	rtc_get_datetime(dt);
}

void rtc_get_datetime( rtc_time_t *dt )
{
	uint32_t year = 0;
	uint32_t month = 0;
	uint32_t week = 0;
	uint32_t day = 0;
	uint32_t hour = 0;
	uint32_t minute = 0;
	uint32_t sec = 0;
	
	rtc_get_date(RTC, &year, &month, &day, &week);
	rtc_get_time(RTC, &hour, &minute, &sec);
	
	dt->century = 20;
	dt->year = (uint8_t)(year - 2000);
	dt->month = (uint8_t)month;
	dt->day = (uint8_t)day;
	dt->hour = (uint8_t)hour;
	dt->minute = (uint8_t)minute;
	dt->second = (uint8_t)sec;
	
	//printf("RTC set to %d/%02d/%02d %02d:%02d:%02d\r\n", year, month, day, hour, minute, sec);

}

void rtc_get_epoch( uint32_t *epoch )
{
	uint32_t year = 0;
	uint32_t month = 0;
	uint32_t week = 0;
	uint32_t day = 0;
	uint32_t hour = 0;
	uint32_t minute = 0;
	uint32_t sec = 0;
	
	rtc_get_date(RTC, &year, &month, &day, &week);
	rtc_get_time(RTC, &hour, &minute, &sec);
	
	*epoch = time_to_epoch(year-2000, month, day, hour, minute, sec);

	//printf("RTC set to %d/%02d/%02d %02d:%02d:%02d\r\n", year, month, day, hour, minute, sec);

}


