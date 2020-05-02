/*
 * ds3231.h
 *
 * Created: 7/27/2018
 *  Author: Chris
 */ 


#ifndef DS3231_H_
#define DS3231_H_

#include "rtc_time.h"

/*
 * Responses from RTCReadSignature()
 */
typedef enum
{
	SIGNATURE_NOT_VALID,	// Device is not configured or has completely lost power
	SIGNATURE_DATETIME_SET,	// Device has been configured and RTCSetDateTime() has been called
	SIGNATURE_DATETIME_NOT_SET	// Device has been configured and RTCSetDateTime() has NOT been called
} ds3231_status;

/*
 * Simple date/time structure passed to RTCGetDateTime(), RTCIsDateTimeValid(), RTCSetDateTime()
 */
//typedef struct
//{
//	uint8_t	century;
//	uint8_t	year;
//	uint8_t	month;
//	uint8_t	day;
//	uint8_t	hour;
//	uint8_t	minute;
//	uint8_t	second;
//} ds3231_datetime;

typedef rtc_time_t ds3231_datetime;

extern uint32_t ds3231_twi_init(void);
extern uint32_t ds3231_write(uint16_t addr, uint8_t *buf, uint16_t len);
extern uint32_t ds3231_read(uint16_t addr, uint8_t *buf, uint16_t len);
extern uint32_t ds3231_read_register( uint8_t registerNumber, uint8_t *byte);
extern uint32_t ds3231_write_register( uint8_t registerNumber, uint8_t *byte);

/* Public Functions */
extern uint32_t ds3231_init(void);
extern uint32_t ds3231_get_datetime( ds3231_datetime *dt);
extern uint32_t ds3231_set_datetime( ds3231_datetime *dt);
extern uint32_t ds3231_set_alarm( ds3231_datetime *dt );
extern uint32_t ds3231_datetime_valid(const ds3231_datetime *dt);

// replaced with epoch.c functions
//extern unsigned int ds3231_datetime_to_epoch(ds3231_datetime *dt);
//extern void ds3231_epoch_to_datetime(ds3231_datetime *dt, unsigned int epoch);


#endif /* DS3231_H_ */