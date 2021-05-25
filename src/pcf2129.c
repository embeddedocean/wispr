/*
 * pcf2129.c
 *
 * Created: 1/2021
 * Author: Chris Jones
 */ 

#include	<asf.h>
#include	<assert.h>
#include	<string.h>

#include "pcf2129.h"

// constants
#define PCF2129_ADDR  (0xA2 >> 1)

static enum pcf2129_reg
{
	// Control Register
	CONTROL_1_REGISTER		= 0x00,
	CONTROL_2_REGISTER		= 0x01,
	CONTROL_3_REGISTER		= 0x02,
	CLKOUT_REGISTER			= 0x0F,
	WDT_CTL_REGISTER		= 0x10,

	// Time and data Register
	SECONDS_REGISTER		= 0x03,
	MINUTES_REGISTER		= 0x04,
	HOURS_REGISTER			= 0x05,
	DAYS_REGISTER			= 0x06,
	WEEKDAYS_REGISTER		= 0x07,
	MONTHS_REGISTER			= 0x08,
	YEARS_REGISTER			= 0x09,

	// Alarm Regs
	ALARM_SECONDS_REGISTER	= 0x0A,
	ALARM_MINUTES_REGISTER	= 0x0B,
	ALARM_HOURS_REGISTER	= 0x0C,
	ALARM_DAYS_REGISTER		= 0x0D,
	ALARM_WEEKDAYS_REGISTER	= 0x0D,
	
	// Clkout Control Register
	CLKOUT_CTL_REGISTER		= 0x0F,

	/*
	* Miscellaneous
	*/
	TIMEMEOUT = 1000,	// loop count
};

/*
 * DS3231 Registers
 *
 * Control and Status Registers
 */
typedef struct 
{
	uint8_t si		 : 1;	// 
	uint8_t mi		 : 1;	// 
	uint8_t mode	 : 1;	// 
	uint8_t por_ovrd : 1;	// 
	uint8_t tsf1	 : 1;	// 
	uint8_t stop	 : 1;	// 
	uint8_t t		 : 1;	// 
	uint8_t ext_test : 1;	// 
} pcf2129_control_1_register;

typedef struct
{
	uint8_t t1		: 1;	//
	uint8_t aie		: 1;	//
	uint8_t tsie	: 1;	//
	uint8_t t2		: 1;	//
	uint8_t af		: 1;	//
	uint8_t tsf2	: 1;	//
	uint8_t wdtf	: 1;	//
	uint8_t msf		: 1;	//
} pcf2129_control_2_register;

typedef struct
{
	uint8_t pwrmng		: 3;	//
	uint8_t btse		: 1;	//
	uint8_t bf			: 1;	//
	uint8_t blf			: 1;	//
	uint8_t bie			: 1;	//
	uint8_t blie		: 1;	//
} pcf2129_control_3_register;

typedef struct
{
	uint8_t tcr		: 2;	//
	uint8_t otpr	: 1;	//
	uint8_t unused	: 2;	//
	uint8_t cof		: 3;	//
} pcf2129_clkout_ctl_register;

typedef struct
{
	uint8_t wd_cd	: 1;	//
	uint8_t t		: 1;	//
	uint8_t ti_tp	: 1;	//
	uint8_t unused	: 3;	//
	uint8_t tf		: 2;	//
} pcf2129_wdt_ctl_register;

/*
 * the registers for the date and time information generally contain the data in bcd. the 10's values may not be a full
 * four bits, some of the registers also have flags in them.
 */
typedef struct 
{
	uint8_t unit	: 4; // bcd unit place
	uint8_t tens	: 3; // bcd ten's place
	uint8_t osf		: 1; // clock integrity (0 is good)
} pcf2129_seconds_register;

typedef struct 
{
	uint8_t unit	: 4;
	uint8_t tens	: 3;
	uint8_t unused	: 1;
} pcf2129_minutes_register;

// only uses 24 hour mode
typedef struct 
{
	uint8_t unit	: 4; 
	uint8_t tens	: 2;
	uint8_t unused	: 2;  
} pcf2129_hours_register;

typedef struct
{
	uint8_t unit	: 4;
	uint8_t tens	: 2;
	uint8_t unused	: 2;
} pcf2129_days_register;

typedef struct
{
	uint8_t unit	: 3;
	uint8_t unused	: 5;  
} pcf2129_weekdays_register;


typedef struct
{
	uint8_t unit	: 3;
	uint8_t tens	: 1;
	uint8_t unused	: 4; 
} pcf2129_months_register;

typedef struct 
{
	uint8_t unit	: 4;
	uint8_t tens	: 4;
} pcf2129_years_register;

typedef struct 
{
	pcf2129_seconds_register	seconds;
	pcf2129_minutes_register	minutes;
	pcf2129_hours_register		hours;
	pcf2129_days_register		days;
	pcf2129_weekdays_register	weekdays;
	pcf2129_months_register		months;
	pcf2129_years_register		years;
} pcf2129_datetime_registers;

typedef struct 
{
	uint8_t	startregister;
	pcf2129_datetime_registers	dt;
} pcf2129_datetime_transmission_packet;
 

// Alarm registers for the date and time generally contain the data in binary coded decimal (bcd). 
// The number of bit in each digit varies and some registers contains flags.

typedef struct 
{
	uint8_t unit	: 4;
	uint8_t tens	: 3;
	uint8_t enabled	: 1;
} pcf2129_alarm_seconds_register;

typedef struct 
{
	uint8_t unit	: 4;
	uint8_t tens	: 3;
	uint8_t enabled	: 1;
} pcf2129_alarm_minutes_register;

typedef struct 
{
	uint8_t unit	: 4;
	uint8_t tens	: 2;
	uint8_t unused	: 1;
	uint8_t enabled	: 1;
} pcf2129_alarm_hours_register;

typedef struct 
{
	uint8_t unit	: 4;
	uint8_t tens	: 2;
	uint8_t unused	: 1;
	uint8_t enabled	: 1;
} pcf2129_alarm_days_register;

typedef struct
{
	uint8_t unit	: 3;
	uint8_t unused	: 4;
	uint8_t enabled	: 1;
} pcf2129_alarm_weekdays_register;

typedef struct
{
	pcf2129_alarm_seconds_register	seconds;
	pcf2129_alarm_minutes_register	minutes;
	pcf2129_alarm_hours_register	hours;
	pcf2129_alarm_days_register		days;
} pcf2129_alarm_registers;
 
typedef struct
{
	uint8_t	start_register;
	pcf2129_alarm_registers	alarm;
} pcf2129_alarm_transmission_packet;


/****************************************************************************************************************************/
// TWI functions

Twi *PCF2129_TWI = TWI0;

uint32_t pcf2129_twi_init(void)
{
	uint32_t status = twi_probe(PCF2129_TWI, PCF2129_ADDR);
	if( status != TWI_SUCCESS ) {
		printf("I2C probe FAILED: 0x%x\r\n", status);
	}
	
	return(status);
}

uint32_t pcf2129_write(uint16_t addr, uint8_t *buf, uint16_t len)
{
	struct twi_packet p;
	p.addr[0] = 0;
	p.addr_length = 0;
	p.buffer = buf;
	p.length = len;
	p.chip = addr;

	uint32_t status = twi_master_write(PCF2129_TWI, &p);

	return(status);	
}

uint32_t pcf2129_read(uint16_t addr, uint8_t *buf, uint16_t len)
{
	struct twi_packet p;
	p.addr[0] = 0;
	p.addr_length = 0;
	p.buffer = buf;
	p.length = len;
	p.chip = addr;

	uint32_t status = twi_master_read(PCF2129_TWI, &p);

	return(status);
}

/****************************************************************************************************************************/
//
// DS3231 functions

#define ASBYTE(b)       (*((uint8_t*) &(b)))

static bool	pcf2129_configured = false;


uint32_t pcf2129_read_register( uint8_t registerNumber, uint8_t *byte)
{
	uint32_t status = ~TWI_SUCCESS;

	/*
	 * Send the register number we are trying to read to the device. We do NOT want send a stop flag or the transaction
	 * will end up completed before we try to read the register data
	 */
	if ((status = pcf2129_write(PCF2129_ADDR, &registerNumber, sizeof(registerNumber))) != TWI_SUCCESS) {
		return status;
	}

	if ((status = pcf2129_read(PCF2129_ADDR, byte, sizeof(uint8_t))) != TWI_SUCCESS) {
		return status;
	}

	return status;
}
 
uint32_t pcf2129_write_register( uint8_t registerNumber, uint8_t *byte)
{
	uint8_t data[2];
	uint32_t status = ~TWI_SUCCESS;

	/*
	 * Send the register number we are trying to write to the device followed by the data to be written.
	 */
	data[0] = registerNumber;
	data[1] = byte[0];

	status = pcf2129_write(PCF2129_ADDR, data, 2);

	return status;
}
 
 
uint32_t pcf2129_init(void)
{
	uint32_t status = RTC_STATUS_OK;
	uint8_t buf[4]; // packet contains start address and 3 regs
	
	status = twi_probe(PCF2129_TWI, PCF2129_ADDR);
	if( status != TWI_SUCCESS ) {
		printf("pcf2129_initialize(): I2C probe FAILED: 0x%x\r\n", status);
		return RTC_REG_ERR;
	}
	
	buf[0] = CONTROL_1_REGISTER; // starting address for auto-incrementing address write operation
	buf[1] = 0x01; 	// control 1 register: SI = 1; - enable second interrupt for PPS
	buf[2] = 0x00; 	// control 2 register:
	buf[3] = 0x00; // control 3 register

	// write ctrl register packet
	//pcf2129_write(PCF2129_ADDR, buf, 4);
	if ((status = pcf2129_write(PCF2129_ADDR, buf, 4)) != TWI_SUCCESS) {
		printf("pcf2129_init: pcf2129_write failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}

	// read back the ctrl register to check it 
	if ((status = pcf2129_write(PCF2129_ADDR, buf, 1)) != TWI_SUCCESS) {
		printf("pcf2129_init: pcf2129_write failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	if ((status = pcf2129_read(PCF2129_ADDR, &buf[1], 3)) != TWI_SUCCESS) {
		printf("pcf2129_init: pcf2129_write failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	// write wdt ctl register to set pps on int
	buf[0] = WDT_CTL_REGISTER; // starting address for auto-incrementing address write operation
	buf[1] = 0x23; // ti_tp = 1, TF[1:0] = 11;	// see figure 22 of datasheet
	//buf[1] = 0x03; // ti_tp = 0, TF[1:0] = 11;	// see figure 22 of datasheet

	if ((status = pcf2129_write(PCF2129_ADDR, buf, 2)) != TWI_SUCCESS) {
		printf("pcf2129_initialize(): pcf2129_write_register(wdt_ctl_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	
	// write wdt ctl register to set pps on int
	buf[0] = CLKOUT_CTL_REGISTER; // starting address for auto-incrementing address write operation
	buf[1] = 0x00; // 

	if ((status = pcf2129_write(PCF2129_ADDR, buf, 2)) != TWI_SUCCESS) {
		printf("pcf2129_initialize(): pcf2129_write_register(wdt_ctl_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	pcf2129_configured = true;
	
	return status;
}

uint32_t pcf2129_set_alarm( pcf2129_datetime *dt )
{
	uint32_t status = ~TWI_SUCCESS;
	uint8_t buf[4]; // packet contains start address and 3 regs
	pcf2129_alarm_transmission_packet packet;
	
	buf[0] = CONTROL_1_REGISTER; // starting address for auto-incrementing address write operation
	buf[1] = 0x00; 	// control 1 register:
	buf[2] = 0x02; 	// control 2 register: AIE = 1 enable interrupt when alarm flag set
	buf[3] = 0x00; // control 3 register

	// write ctrl register packet
	if ((status = pcf2129_write(PCF2129_ADDR, buf, 4)) != TWI_SUCCESS) {
		printf("pcf2129_init: pcf2129_write failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}

	// clear pack to set the unused bits to zero
	memset(&packet, 0, sizeof(packet));
	
	// set alarm registers
	packet.start_register = ALARM_SECONDS_REGISTER;

	// Convert alarm datetime to BCD
	
	packet.alarm.seconds.unit	= dt->second % 10;
	packet.alarm.seconds.tens	= dt->second / 10;

	packet.alarm.minutes.unit	= dt->minute % 10;
	packet.alarm.minutes.tens	= dt->minute / 10;

	packet.alarm.hours.unit = dt->hour % 10;
	packet.alarm.hours.tens = dt->hour / 10;

	packet.alarm.days.unit	= dt->day % 10;
	packet.alarm.days.tens	= dt->day / 10;

	// enable alarms
	packet.alarm.seconds.enabled = 0; 
	packet.alarm.minutes.enabled = 0; 
	packet.alarm.hours.enabled = 0;
	packet.alarm.days.enabled = 0;

	// Write the alarm packet to the device
	if ((status = pcf2129_write(PCF2129_ADDR, (uint8_t*) &packet, sizeof(packet))) != TWI_SUCCESS) {
		printf("pcf2129_set_alarm() setting alarm failed with status 0x%02x\r\n",status);
		return(RTC_REG_ERR);
	}
			
	return status;
}
 


 /****************************************************************************************************************************
 **
 ** rtc_get_datetime()
 **/
uint8_t  pcf2129_baseyear = 20;

uint32_t pcf2129_get_datetime( pcf2129_datetime*  dt)
{
	pcf2129_datetime_registers datetime;
	uint8_t secondsregister = SECONDS_REGISTER;
	uint32_t status = 0;
	
	/* assume failure */
	dt->year		= 0;
	dt->month		= 0;
	dt->day			= 0;
	dt->hour		= 0;
	dt->minute		= 0;
	dt->second		= 0;
	dt->century		= 20;
	
//	pcf2129_statusregister reg;
//	if ((status = pcf2129_read_register(STATUS_REGISTER, (uint8_t *)&reg)) != TWI_SUCCESS)
//	{
//		printf("pcf2129_get_datetime(): pcf2129_read_register(status_register) failed with status 0x%02x\r\n",status);
//		return RTC_REG_ERR;
//	}
	
	// check if RTC osc has stopped for any reason
//	if( reg.osf == 1 ) {
//		printf("pcf2129_get_datetime(): status_register osf bit set, invalid date\r\n");
//		return RTC_REG_ERR;
//	}
	 
	/* read the time from the device */
	if ((status = pcf2129_write(PCF2129_ADDR, &secondsregister, 1)) != TWI_SUCCESS)
	{
		printf("pcf2129_get_datetime() selecting the seconds register failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	memset(&datetime, 0, sizeof(datetime));
	
	if ((status = pcf2129_read(PCF2129_ADDR, (uint8_t*) &datetime, 7)) != TWI_SUCCESS)
	{
		printf("pcf2129_get_datetime() reading the date/time data failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	/* convert the bsd response to a simple date time structure */
	
	dt->second = (datetime.seconds.tens * 10) + datetime.seconds.unit;
	dt->minute = (datetime.minutes.tens * 10) + datetime.minutes.unit;
	dt->hour = (datetime.hours.tens * 10) + datetime.hours.unit;	
	dt->day = (datetime.days.tens * 10) + datetime.days.unit;
	dt->month = (datetime.months.tens * 10) + datetime.months.unit;
	dt->year = (datetime.years.tens * 10) + datetime.years.unit;

	dt->century	= 20;
		
	status = RTC_STATUS_OK;
	
	return status;
}

/****************************************************************************************************************************
 **
 ** pcf2129_set_data_time()
 **/

uint32_t pcf2129_set_datetime( pcf2129_datetime*  dt)
{
	uint32_t status = RTC_STATUS_OK;
	pcf2129_datetime_transmission_packet packet;

	// Confirm that the date/time information that we were supplied is valid
	status = pcf2129_datetime_valid(dt);
	if ( status != RTC_STATUS_OK ) {
		printf("pcf2129_set_datetime() has been called with an invalid date/time\r\n");
		return(status);
	}

	// clear pack to set the unused bits to zero
	memset(&packet, 0, sizeof(packet));

	// Convert decimal data and time to BCD

	packet.dt.seconds.unit = dt->second % 10;
	packet.dt.seconds.tens = dt->second / 10;
	packet.dt.seconds.osf = 0;

	packet.dt.minutes.unit = dt->minute % 10;
	packet.dt.minutes.tens = dt->minute / 10;

	packet.dt.hours.unit = dt->hour % 10;
	packet.dt.hours.tens = dt->hour / 10;
	
	packet.dt.days.unit = dt->day % 10;
	packet.dt.days.tens = dt->day / 10;

	packet.dt.months.unit = dt->month % 10;
	packet.dt.months.tens = dt->month / 10;

	packet.dt.years.unit = dt->year % 10;
	packet.dt.years.tens = dt->year / 10;

    // select the starting register for the multiple register read operation
    packet.startregister = SECONDS_REGISTER;

	// Write the date/time to the device
	if ((status = pcf2129_write(PCF2129_ADDR, (uint8_t*) &packet, sizeof(packet))) != TWI_SUCCESS) {
		printf("pcf2129_set_datetime() storing the date/time to the device failed with status 0x%02x\r\n",status);
		status = RTC_REG_ERR;
	}
	
	return status;
}



/****************************************************************************************************************************
 **
 **/

static uint8_t daysPerMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

uint32_t pcf2129_datetime_valid(const pcf2129_datetime *dt)
{
	uint32_t status = RTC_STATUS_OK;

	/*
	 * Check each field to make sure that it is in range
	 *
	 * -- Milliseconds are 0 to 999
	 */
	//if (dt->millisecond > 999)
	//{
	//	goto EXIT;
	//}

	/*
	 * -- Seconds are 0 to 59
	 */
	if (dt->second > 59) {
		status |= RTC_INVALID_SECOND;
	}

	/*
	 * -- Minutes are 0 to 59
	 */
	if (dt->minute > 59) {
		status |= RTC_INVALID_MINUTE;
	}

	/*
	 * Hours are 0 to 23
	 */
	if (dt->hour > 23) {
		status |= RTC_INVALID_HOUR;
	}

	/*
	 * Months are 1 to 12
	 */
	if ((dt->month == 0) || (dt->month > 12)) {
		status |= RTC_INVALID_MONTH;
	}

	/*
	 * The day of the month is based on the number of days in the month and whether or not the year is a leap year
	 */
	uint8_t	days;
	if (dt->month == 2)	{					// February
		if ((dt->year % 400) == 0) {		// Years which are multiples of 400 are leap years
			days = 29;
		}
		else if ((dt->year % 100) == 0)	{	// Years which are multiples of 100 (but not 400) are not leap years
			days = 28;
		}
		else if ((dt->year % 4) == 0) {		// "Normal" leap years
			days = 29;
		}
		else {								// The only case left is non leap years
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

	if ((dt->year < 0) || (dt->year > 99)) {
		status |= RTC_INVALID_YEAR;
	}

	/*
	 * All checks have passed
	 */

	return(status);
}

/*
uint32_t pcf2129_init2(void)
{
	uint32_t status = RTC_STATUS_OK;
	
	status = twi_probe(PCF2129_TWI, PCF2129_ADDR);
	if( status != TWI_SUCCESS ) {
		printf("pcf2129_initialize(): I2C probe FAILED: 0x%x\r\n", status);
		return RTC_REG_ERR;
	}

	uint8_t ctrl[4];

	// write control 1 register
	ctrl[0] = 0x01; 	// ctrl1.si = 1; - enable second interrupt for PPS

	// write control 2 register
	ctrl[1] = 0x80; 	// MSF = 1

	// write control 3 register
	ctrl[2] = 0x00;

	if ((status = pcf2129_write_register(CONTROL_1_REGISTER, (uint8_t *)&ctrl[0])) != TWI_SUCCESS)
	{
		printf("pcf2129_initialize(): pcf2129_write_register(control_1_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	if ((status = pcf2129_write_register(CONTROL_2_REGISTER, (uint8_t *)&ctrl[1])) != TWI_SUCCESS)
	{
		printf("pcf2129_initialize(): pcf2129_write_register(control_2_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	// write control 3 register
	if ((status = pcf2129_write_register(CONTROL_3_REGISTER, (uint8_t *)&ctrl[2])) != TWI_SUCCESS)
	{
		printf("pcf2129_initialize(): pcf2129_write_register(control_3_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	// write wdt ctl register to control pps on int
	//pcf2129_wdt_ctl_register wdt;
	//wdt.wd_cd = 0;	//
	//wdt.t = 0;	//
	//wdt.ti_tp = 1;	// see figure 22 of datasheet
	//wdt.unused = 0;
	//wdt.tf = 0;	//
	uint8_t reg = 0x23;
	if ((status = pcf2129_write_register(WDT_CTL_REGISTER, (uint8_t *)&reg)) != TWI_SUCCESS)
	{
		printf("pcf2129_initialize(): pcf2129_write_register(wdt_ctl_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	pcf2129_configured = true;	
	 
	return status;
}

uint32_t pcf2129_set_alarm2( pcf2129_datetime *dt )
{
	pcf2129_alarm_transmission_packet packet;
	uint32_t	status = ~TWI_SUCCESS;

	uint8_t reg = 0;

	// write control 1 register
	// enable SI - clear second interrupt
	reg = 0x00;
	if ((status = pcf2129_write_register(CONTROL_1_REGISTER, (uint8_t *)&reg)) != TWI_SUCCESS)
	{
		printf("pcf2129_set_alarm(): pcf2129_write_register(control_1_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	// write control 2 register
	//ctrl2.aie = 1; // interrupt generated when alarm flag set
	reg = 0x02;
	if ((status = pcf2129_write_register(CONTROL_2_REGISTER, (uint8_t *)&reg)) != TWI_SUCCESS)
	{
		printf("pcf2129_set_alarm(): pcf2129_write_register(control_2_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}
	
	// write control 3 register
	reg = 0x00;
	if ((status = pcf2129_write_register(CONTROL_3_REGISTER, (uint8_t *)&reg)) != TWI_SUCCESS)
	{
		printf("pcf2129_set_alarm(): pcf2129_write_register(control_3_register) failed with status 0x%02x\r\n", status);
		return RTC_REG_ERR;
	}

	// set alarm registers
	packet.start_register = ALARM_SECONDS_REGISTER;

	// Convert alarm datetime to BCD
	
	packet.alarm.seconds.unit	= dt->second % 10;
	packet.alarm.seconds.tens	= dt->second / 10;

	packet.alarm.minutes.unit	= dt->minute % 10;
	packet.alarm.minutes.tens	= dt->minute / 10;

	packet.alarm.hours.unit = dt->hour % 10;
	packet.alarm.hours.tens = dt->hour / 10;

	packet.alarm.days.unit	= dt->day % 10;
	packet.alarm.days.tens	= dt->day / 10;

	// enable alarms
	packet.alarm.seconds.enabled = 1; 
	packet.alarm.minutes.enabled = 1; 
	packet.alarm.hours.enabled = 1;
	packet.alarm.days.enabled = 1;

	// Write the alarm packet to the device
	if ((status = pcf2129_write(PCF2129_ADDR, (uint8_t*) &packet, sizeof(packet))) != TWI_SUCCESS)
	{
		printf("pcf2129_set_alarm() setting alarm failed with status 0x%02x\r\n",status);
		return(RTC_REG_ERR);
	}

			
	return status;
}
 */