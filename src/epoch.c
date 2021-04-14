/*
 * epoch.c
 *
 * Created: 1/16/2020 8:41:11 AM
 *  Author: Chris
 */ 

//---------------------------------------------------------------------------------
// epoch calculator good for years 0-99 (representing 2000-2099)
// Found this on stackexchange

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "rtc_time.h"
#include "epoch.h"

// offset is January 1, 2000 12:00:00 AM
#define EPOCH_OFFSET 946684800

static uint32_t DAYS[4][12] =
{
	{   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
	{ 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
	{ 731, 762, 790, 821, 851, 882, 912, 943, 974,1004,1035,1065},
	{1096,1127,1155,1186,1216,1247,1277,1308,1339,1369,1400,1430},
};


uint32_t rtc_time_to_epoch(rtc_time_t *t)
{
	uint32_t sec = (uint32_t)t->second;  // 0-59
	uint32_t minute = (uint32_t)t->minute;  // 0-59
	uint32_t hour   = (uint32_t)t->hour;    // 0-23
	uint32_t day    = (uint32_t)t->day-1;   // 0-30
	uint32_t month  = (uint32_t)t->month-1; // 0-11
	uint32_t year   = (uint32_t)t->year;    // 0-99
	return (((year/4*(365*4+1)+DAYS[year%4][month]+day)*24+hour)*60+minute)*60+sec + EPOCH_OFFSET;
}

uint32_t time_to_epoch(uint8_t yr, uint8_t mn, uint8_t dy, uint8_t hr, uint8_t mi, uint8_t se)
{
	uint32_t sec    = (uint32_t)se;    // 0-59
	uint32_t minute = (uint32_t)mi;    // 0-59
	uint32_t hour   = (uint32_t)hr;    // 0-23
	uint32_t day    = (uint32_t)dy-1;  // 0-30
	uint32_t month  = (uint32_t)mn-1;  // 0-11
	uint32_t year   = (uint32_t)yr;    // 0-99
	return (((year/4*(365*4+1)+DAYS[year%4][month]+day)*24+hour)*60+minute)*60+sec + EPOCH_OFFSET;
}


void epoch_to_rtc_time(rtc_time_t *t, uint32_t epoch)
{
	epoch = epoch - EPOCH_OFFSET;
	
	t->second = (uint8_t)(epoch%60);
	epoch /= 60;
	t->minute = (uint8_t)(epoch%60);
	epoch /= 60;
	t->hour   = (uint8_t)(epoch%24);
	epoch /= 24;

	uint32_t years = epoch/(365*4+1)*4;
	epoch %= 365*4+1;

	uint32_t year;
	for (year=3; year>0; year--)
	{
		if (epoch >= DAYS[year][0])
		break;
	}

	uint8_t month;
	for (month=11; month>0; month--)
	{
		if (epoch >= DAYS[year][month])
		break;
	}

	t->tzoff  = 0; // always UTC
	t->century  = 20; // always 20th century
	t->year  = (uint8_t)(years + year);
	t->month = (uint8_t)(month + 1);
	t->day   = (uint8_t)(epoch-DAYS[year][month] + 1);
}

void epoch_to_time(uint32_t epoch, uint8_t *yr, uint8_t *mn, uint8_t *dy, uint8_t *hr, uint8_t *mi, uint8_t *se)
{
	epoch = epoch - EPOCH_OFFSET;
	
	*se = (uint8_t)(epoch%60);
	epoch /= 60;
	*mi = (uint8_t)(epoch%60);
	epoch /= 60;
	*hr   = (uint8_t)(epoch%24);
	epoch /= 24;

	uint32_t years = epoch/(365*4+1)*4;
	epoch %= 365*4+1;

	uint32_t year;
	for (year=3; year>0; year--)
	{
		if (epoch >= DAYS[year][0])
		break;
	}

	uint8_t month;
	for (month=11; month>0; month--)
	{
		if (epoch >= DAYS[year][month])
		break;
	}

	//century  = 20; // always 20th century
	*yr  = (uint8_t)(years + year);
	*mn = (uint8_t)(month + 1);
	*dy   = (uint8_t)(epoch-DAYS[year][month] + 1);
}

void epoch_to_time_string(uint32_t epoch, char *str)
{
	uint8_t yr, mn, dy, hr, mi, se;
	epoch_to_time(epoch, &yr, &mn, &dy, &hr, &mi, &se);
	sprintf(str, "%02d/%02d/%02d %02d:%02d:%02d\r\n", yr, mn, dy, hr, mi, se);
}

char time_string[20];
char *epoch_time_string(uint32_t epoch)
{
	rtc_time_t tme;
	epoch_to_rtc_time(&tme, epoch);
	sprintf(time_string, "%02d/%02d/%02d %02d:%02d:%02d",
		tme.year,tme.month,tme.day,tme.hour,tme.minute,tme.second);
	//	uint8_t yr, mn, dy, hr, mi, se;
	//	epoch_to_time(epoch, &yr, &mn, &dy, &hr, &mi, &se);
	//	sprintf(time_string, "%02d/%02d/%02d %02d:%02d:%02d", yr, mn, dy, hr, mi, se);
	return(time_string);
}

uint32_t time_string_to_epoch(char *str)
{
	int yr, mn, dy, hr, mi, se;
	sscanf(str, "%02d/%02d/%02d %02d:%02d:%02d", &yr, &mn, &dy, &hr, &mi, &se);
	//uint32_t epoch = rtc_time_to_epoch(&tme);
	uint32_t epoch = time_to_epoch(yr, mn, dy, hr, mi, se);
	return(epoch);
}

