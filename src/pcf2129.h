/*
 * pcf2129.h
 *
 * Created: 7/27/2018
 *  Author: Chris
 */ 


#ifndef PCF2129_H_
#define PCF2129_H_

#include "rtc_time.h"

/*
 * Responses from RTCReadSignature()
 */
typedef enum
{
	SIGNATURE_NOT_VALID,	// Device is not configured or has completely lost power
	SIGNATURE_DATETIME_SET,	// Device has been configured and RTCSetDateTime() has been called
	SIGNATURE_DATETIME_NOT_SET	// Device has been configured and RTCSetDateTime() has NOT been called
} pcf2129_status;

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
//} pcf2129_datetime;

typedef rtc_time_t pcf2129_datetime;

extern uint32_t pcf2129_twi_init(void);
extern uint32_t pcf2129_write(uint16_t addr, uint8_t *buf, uint16_t len);
extern uint32_t pcf2129_read(uint16_t addr, uint8_t *buf, uint16_t len);
extern uint32_t pcf2129_read_register( uint8_t registerNumber, uint8_t *byte);
extern uint32_t pcf2129_write_register( uint8_t registerNumber, uint8_t *byte);

/* Public Functions */
extern uint32_t pcf2129_init(void);
extern uint32_t pcf2129_get_datetime( pcf2129_datetime *dt);
extern uint32_t pcf2129_set_datetime( pcf2129_datetime *dt);
extern uint32_t pcf2129_set_alarm( pcf2129_datetime *dt );
extern uint32_t pcf2129_datetime_valid(const pcf2129_datetime *dt);

// replaced with epoch.c functions
//extern unsigned int pcf2129_datetime_to_epoch(pcf2129_datetime *dt);
//extern void pcf2129_epoch_to_datetime(pcf2129_datetime *dt, unsigned int epoch);


#endif /* PCF2129_H_ */