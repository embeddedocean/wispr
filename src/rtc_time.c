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

/* The BCD code shift value */
#define BCD_SHIFT      4

/* The BCD code mask value */
#define BCD_MASK       0xfu

/* The BCD mul/div factor value */
#define BCD_FACTOR     10

// msec to wait for valid time
#define RTC_TIMEOUT 2000

uint32_t rtc_init( rtc_time_t *dt )
{	
	rtc_clear_status(RTC, 0xFFFF);
	rtc_clear_time_alarm(RTC);
	rtc_clear_date_alarm(RTC);
	rtc_disable_interrupt(RTC, 0xFFFF);
	
	//uint32_t usec = 0;
	
	rtc_set_hour_mode(RTC, 0);
	
	uint32_t status = rtc_set_datetime(dt);
	if(status != RTC_STATUS_OK) return(status);

	//printf("Internal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n", year, month, day, hour, minute, sec);
	
	// read time back to verify
	status = rtc_get_datetime(dt);
	
	return(status);
}


uint32_t rtc_get_epoch( uint32_t *epoch )
{
	rtc_time_t dt;
	uint32_t status = rtc_get_datetime(&dt);
	*epoch = rtc_time_to_epoch(&dt);
	//printf("RTC set to %d/%02d/%02d %02d:%02d:%02d\r\n", year, month, day, hour, minute, sec);
	return(status);
}

void rtc_get_datetime_asf( rtc_time_t *dt )
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

//
// Get the RTC data and time
// Returns the RTC Valid Entry Register NVTIM: Non-valid Time
// 0 = Success.
// 1 = RTC_TIMR has contained invalid data since it was last programmed.
// 2 = RTC_CALR has contained invalid data since it was last programmed.
// 3 = Both contained invalid data.
// 4 = Invalid date or time provided
//
uint32_t rtc_get_datetime( rtc_time_t *dt )
{
	Rtc *p_rtc = RTC;
	uint32_t year, month, day, week; 
	uint32_t hour, minute, second;
	
	uint32_t ul_date;
	uint32_t ul_cent;
	uint32_t ul_temp;

	/* Get the current date (multiple reads are necessary to insure a stable value). */
	ul_date = p_rtc->RTC_CALR;
	while (ul_date != p_rtc->RTC_CALR) {
		ul_date = p_rtc->RTC_CALR;
	}

	/* Retrieve year */
	ul_temp = (ul_date & RTC_CALR_CENT_Msk) >> RTC_CALR_CENT_Pos;
	ul_cent = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);
	ul_temp = (ul_date & RTC_CALR_YEAR_Msk) >> RTC_CALR_YEAR_Pos;
	year = (ul_cent * BCD_FACTOR * BCD_FACTOR) +
	(ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);

	/* Retrieve month */
	ul_temp = (ul_date & RTC_CALR_MONTH_Msk) >> RTC_CALR_MONTH_Pos;
	month = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);

	/* Retrieve day */
	ul_temp = (ul_date & RTC_CALR_DATE_Msk) >> RTC_CALR_DATE_Pos;
	day = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);

	/* Retrieve week */
	week = ((ul_date & RTC_CALR_DAY_Msk) >> RTC_CALR_DAY_Pos);

	uint32_t ul_time;

	/* Get the current RTC time (multiple reads are necessary to insure a stable value). */
	ul_time = p_rtc->RTC_TIMR;
	while (ul_time != p_rtc->RTC_TIMR) {
		ul_time = p_rtc->RTC_TIMR;
	}

	/* Hour */
	ul_temp = (ul_time & RTC_TIMR_HOUR_Msk) >> RTC_TIMR_HOUR_Pos;
	hour = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);

	if ((ul_time & RTC_TIMR_AMPM) == RTC_TIMR_AMPM) {
		hour += 12;
	}

	/* Minute */
	ul_temp = (ul_time & RTC_TIMR_MIN_Msk) >> RTC_TIMR_MIN_Pos;
	minute = (ul_temp >> BCD_SHIFT) * BCD_FACTOR +  (ul_temp & BCD_MASK);

	/* Second */
	ul_temp = (ul_time & RTC_TIMR_SEC_Msk) >> RTC_TIMR_SEC_Pos;
	second = (ul_temp >> BCD_SHIFT) * BCD_FACTOR + (ul_temp & BCD_MASK);

	dt->century = 20;
	dt->year = (uint8_t)(year - 2000);
	dt->month = (uint8_t)month;
	dt->day = (uint8_t)day;
	dt->hour = (uint8_t)hour;
	dt->minute = (uint8_t)minute;
	dt->second = (uint8_t)second;

	// check for invalid date or time
	// check for invalid date or time
	uint32_t status = rtc_valid_datetime(dt);
	if( status != RTC_STATUS_OK ) {
		return(status);
	}
	
	if ( p_rtc->RTC_VER & (RTC_VER_NVTIM | RTC_VER_NVCAL) ) {
		return(RTC_REG_ERR);
	}

	return ( status );
}

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

//
// Set RTC date and time
// The hardware checks if one of the time fields is not correct, the data is not loaded into the register/counter 
// and a flag is set in the validity register.
// Returns the RTC Valid Entry Register NVTIM: Non-valid Time
// 0 = Success.
// 1 = RTC_TIMR has contained invalid data since it was last programmed.
// 2 = RTC_CALR has contained invalid data since it was last programmed.
// 3 = Both contained invalid data.
// 4 = Invalid date or time provided
//
uint32_t rtc_set_datetime(rtc_time_t *dt )
{
	Rtc *p_rtc = RTC;
	uint32_t year = (uint32_t)(dt->century*100 + dt->year);
	uint32_t month = (uint32_t)(dt->month);
	uint32_t day = (uint32_t)(dt->day);
	uint32_t hour = (uint32_t)(dt->hour);
	uint32_t minute = (uint32_t)(dt->minute);
	uint32_t second = (uint32_t)(dt->second);

	// check for invalid date or time 
	uint32_t status = rtc_valid_datetime(dt);
	if( status != RTC_STATUS_OK ) {
		return(status);		
	}

	// set date	
	uint32_t ul_date = 0;

	/* Cent */
	ul_date |= ((year / BCD_FACTOR / BCD_FACTOR / BCD_FACTOR) <<
		(RTC_CALR_CENT_Pos + BCD_SHIFT) |
		((year / BCD_FACTOR / BCD_FACTOR) % BCD_FACTOR) <<  RTC_CALR_CENT_Pos);

	/* Year */
	ul_date |= (((year / BCD_FACTOR) % BCD_FACTOR) <<
		(RTC_CALR_YEAR_Pos + BCD_SHIFT)) |
		((year % BCD_FACTOR) << RTC_CALR_YEAR_Pos);

	/* Month */
	ul_date |= ((month / BCD_FACTOR) << (RTC_CALR_MONTH_Pos + BCD_SHIFT)) |
		((month % BCD_FACTOR) << RTC_CALR_MONTH_Pos);

	/* Week */
	uint32_t week = calculate_week(year, month, day);
	ul_date |= (week << RTC_CALR_DAY_Pos);

	/* Day */
	ul_date |= ((day / BCD_FACTOR) << (RTC_CALR_DATE_Pos + BCD_SHIFT)) |
		((day % BCD_FACTOR) << RTC_CALR_DATE_Pos);

	/* Update calendar register. Check the spec for the flow. */
	//cj	while ((p_rtc->RTC_SR & RTC_SR_SEC) != RTC_SR_SEC); // this can hang
	p_rtc->RTC_CR |= RTC_CR_UPDCAL;
	int timeout = RTC_TIMEOUT;
	while (timeout && ((p_rtc->RTC_SR & RTC_SR_ACKUPD) != RTC_SR_ACKUPD) ) {
		delay_ms(1);
		timeout--;
	}
	p_rtc->RTC_SCCR = RTC_SCCR_ACKCLR;
	p_rtc->RTC_CALR = ul_date;
	p_rtc->RTC_CR &= (~RTC_CR_UPDCAL);

	if(timeout <= 0) {
		printf("rtc_set_datetime: set date timeout!\r\n");
		return(0);
	}

	// set time	
	uint32_t ul_time = 0;

	/* If 12-hour mode, set AMPM bit */
	if ((p_rtc->RTC_MR & RTC_MR_HRMOD) == RTC_MR_HRMOD) {
		if (hour > 12) {
			hour -= 12;
			ul_time |= RTC_TIMR_AMPM;
		}
	}

	/* Hour */
	ul_time |= ((hour / BCD_FACTOR) << (RTC_TIMR_HOUR_Pos + BCD_SHIFT)) |
		((hour % BCD_FACTOR) << RTC_TIMR_HOUR_Pos);

	/* Minute */
	ul_time |= ((minute / BCD_FACTOR) << (RTC_TIMR_MIN_Pos + BCD_SHIFT)) |
		((minute % BCD_FACTOR) << RTC_TIMR_MIN_Pos);

	/* Second */
	ul_time |= ((second / BCD_FACTOR) << (RTC_TIMR_SEC_Pos + BCD_SHIFT)) |
		((second % BCD_FACTOR) << RTC_TIMR_SEC_Pos);

	/* Update time register. Check the spec for the flow. */
	//cj	while ((p_rtc->RTC_SR & RTC_SR_SEC) != RTC_SR_SEC); // this can hang
	p_rtc->RTC_CR |= RTC_CR_UPDTIM;
	timeout = RTC_TIMEOUT;
	while (timeout && ((p_rtc->RTC_SR & RTC_SR_ACKUPD) != RTC_SR_ACKUPD) ) {
		delay_ms(1);
		timeout--;
	}
	p_rtc->RTC_SCCR = RTC_SCCR_ACKCLR;
	p_rtc->RTC_TIMR = ul_time;
	p_rtc->RTC_CR &= (~RTC_CR_UPDTIM);

	if(timeout <= 0) {
		printf("rtc_set_datetime: set time timeout!\r\n");
	}

	// return the RTC Valid Entry Register NVTIM: Non-valid Time
	if ( p_rtc->RTC_VER & (RTC_VER_NVTIM | RTC_VER_NVCAL) ) {
		return(RTC_REG_ERR);
	}
	
	return(RTC_STATUS_OK);
}

static uint8_t daysPerMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

uint32_t rtc_valid_datetime(const rtc_time_t *dt)
{
	uint32_t status = 0;

	// Seconds are 0 to 59
	if (dt->second > 59) {
		status |= RTC_INVALID_SECOND;
	}

	// Minutes are 0 to 59
	if (dt->minute > 59) {
		status |= RTC_INVALID_MINUTE;
	}

	// Hours are 0 to 23
	if (dt->hour > 23) {
		status |= RTC_INVALID_HOUR;
	}

	// Months are 1 to 12
	if ((dt->month == 0) || (dt->month > 12)) {
		status |= RTC_INVALID_MONTH;
	}

	// The day of the month is based on the number of days in the month and whether or not the year is a leap year
	uint8_t days;
	if (dt->month == 2) {	// February
		if ((dt->year % 400) == 0)	{		// Years which are multiples of 400 are leap years
			days = 29;
		} 
		else if ((dt->year % 100) == 0) {	// Years which are multiples of 100 (but not 400) are not leap years
			days = 28;
		}
		else if ((dt->year % 4) == 0) {		// "Normal" leap years
			days = 29;
		} else {							// The only case left is non leap years
			days = 28;
		}

		if ((dt->day == 0) || (dt->day > days)) {
			status |= RTC_INVALID_DAY;
		}
	}
	// All months other than February can use the lookup table
	else if ((dt->day == 0) || (dt->day > daysPerMonth[dt->month - 1])) {
		status |= RTC_INVALID_DAY;
	}

	// year < 99
	if (dt->year > 99){
		status |= RTC_INVALID_YEAR;
	}

	// century should always be 20
	if (dt->century != 20){
		status |= RTC_INVALID_YEAR;
	}

	return(status);

}

void rtc_print_error (uint32_t status)
{
	if(status == RTC_STATUS_OK) printf("RTC no error\r\n");
	if(status & RTC_REG_ERR) printf("RTC error: register read/write error\r\n");
	if(status & RTC_INVALID_ARG) printf("RTC error: invalid argument\r\n");
	if(status & RTC_INVALID_SECOND) printf("RTC error: invalid second\r\n");
	if(status & RTC_INVALID_MINUTE) printf("RTC error: invalid minute\r\n");
	if(status & RTC_INVALID_HOUR) printf("RTC error: invalid hour\r\n");
	if(status & RTC_INVALID_DAY) printf("RTC error: invalid day\r\n");
	if(status & RTC_INVALID_MONTH) printf("RTC error: invalid month\r\n");
	if(status & RTC_INVALID_YEAR) printf("RTC error: invalid year\r\n");
}


