/*
 * ds3231.c
 *
 * Created: 7/27/2018
 * Author: Chris Jones
 */ 

#include	<asf.h>
#include	<assert.h>
#include	<string.h>

#include "ds3231.h"

// constants
#define DS3231_ADDR  (0xD0 >> 1)

static enum
{
	/*
	* Register numbers and field values
	*/
	SECONDS_REGISTER		= 0x00,
	MINUTES_REGISTER		= 0x01,
	HOURS_REGISTER			= 0x02,
	DAY_OF_WEEK_REGISTER	= 0x03,
	DAY_OF_MONTH_REGISTER	= 0x04,
	MONTH_CENTURY_REGISTER	= 0x05,
	YEAR_REGISTER			= 0x06,

	//
	// 0x07 to 0x0A are for Alarm 1
	//
	ALARM1_SECONDS_REGISTER		 = 0x07,
	ALARM1_MINUTES_REGISTER		 = 0x08,
	ALARM1_HOURS_REGISTER		 = 0x09,
	ALARM1_DAY_DATE_REGISTER	 = 0x0A,
	
	// We are using the Alarm 2 registers for device status information. See RTCConfigureDevice()
	//
	ALARM2_MINUTES			= 0x0B,
	SIGNATURE1				= 'I',
	ALARM2_HOURS			= 0x0C,
	SIGNATURE2				= 'V',
	ALARM2_DAY_DATE			= 0x0D,
	BASE_YEAR_2000			= 0,  // 20 * 100
	BASE_YEAR_2500			= 25,  // 25 * 100
	BASE_YEAR_NOT_YET_SET	= 0x5A,  // Something unlikely
	CONTROL_REGISTER		= 0x0E,
	DISABLED				= 0,
	ENABLED					= 1,
	DONT_CARE				= 0,
	HZ_1					= 0,
	HZ_1024					= 1,
	HZ_4096					= 2,
	HZ_8192					= 3,
	STATUS_REGISTER			= 0x0F,
	// 0x10 == Aging offset
	// 0x11-0x12 == Temperature

	/*
	* Miscellaneous
	*/
	TIMEMEOUT = 1000	// Passes through a loop
};

/*
 * DS3231 Registers
 *
 * Control and Status Registers
 */
typedef struct 
{
	uint8_t a1ie	: 1;	// alarm 1 interrupt enable
	uint8_t a2ie	: 1;	// alarm 2 interrupt enable
	uint8_t intcn	: 1;	// interrupt control
	uint8_t rs		: 2;	// rate select for square wave output
	uint8_t conv	: 1;	// convert temperature
	uint8_t bbsqw	: 1;	// battery backed square wave enabled
	uint8_t eoscnot	: 1;	// enable oscillator
} ds3231_controlregister;

typedef struct 
{
	uint8_t a1f		: 1;	// alarm 1 active
	uint8_t a2f		: 1;	// alarm 2 active
	uint8_t bsy		: 1;	// device is busy
	uint8_t en32khz	: 1;	// enable 32khz output
	uint8_t mbz		: 3;	// must be zero
	uint8_t osf		: 1;	// oscillator stopped
} ds3231_statusregister;

/*
 * the registers for the date and time information generally contain the data in bcd. the 10's values may not be a full
 * four bits, some of the registers also have flags in them.
 */
typedef struct 
{
	uint8_t seconds			: 4;  // normal bcd
	uint8_t tensofseconds	: 3;
	uint8_t mbz				: 1;
} ds3231_secondsregister;

typedef struct 
{
	uint8_t minutes			: 4;  // normal bcd
	uint8_t tensofminutes	: 3;
	uint8_t mbz				: 1;
} ds3231_minutesregister;

typedef struct 
{
	uint8_t hour				: 4;	// this one is a little goofy
	uint8_t tensofhours			: 1;	//   when setting the time, if the twelvetwentyfour bit is set to 1, the clock is running
	uint8_t twentiesofhours		: 1;	//   running in 12 hour mode. in that case, twentiesofhours is 1 for pm and 0 for am.
	uint8_t twelvetwentyfour	: 1;	//   if twelvetwentyfour is set to 0, the clock is running in 24 hour mode. see the
	uint8_t mbz					: 1;	//   note below
} ds3231_hoursregister;

	/*
	 * note on the hoursregister
	 *
	 *  twentiesofhours     tensofhours     hour     actual hour
	 *         0                 0          0..9        0..9
	 *      >> 0 <<              1          0..9       10..19
	 *         1              >> 0 <<       0..3       20..23
	 */

typedef struct 
{
	uint8_t day	: 3;	// 1 to 7, it appears that the day of the week value is attached to the month/day/year somehow
	uint8_t mbz	: 5;
} ds3231_dayofweekregister;

typedef struct 
{
	uint8_t day			: 4;	// normal bcd
	uint8_t tensofdays	: 2;
	uint8_t mbz			: 2;
} ds3231_dayofmonthregister;

typedef struct 
{
	uint8_t month			: 4;	// normal bcd
	uint8_t tensofmonths	: 1;
	uint8_t mbz				: 2;
	uint8_t century			: 1;	// when this flag is set, the year value in the clock has rolled over from 99 to 00
} ds3231_monthcenturyregister;

typedef struct 
{
	uint8_t year		: 4;
	uint8_t tensofyears	: 4;
} ds3231_yearregister;

typedef struct 
{
	ds3231_secondsregister		seconds;		// 0x00
	ds3231_minutesregister		minutes;		// 0x01
	ds3231_hoursregister		hours;			// 0x02
	ds3231_dayofweekregister	day;			// 0x03
	ds3231_dayofmonthregister	date;			// 0x04
	ds3231_monthcenturyregister	monthcentury;	// 0x05
	ds3231_yearregister			year;			// 0x06
} ds3231_datetimeregisters;

typedef struct 
{
	uint8_t	startregister;
	ds3231_datetimeregisters	dt;
} ds3231_datetimetransmissionpacket;
 
typedef struct
{
	uint8_t day			: 4;	// normal bcd
	uint8_t tensofdays	: 2;
	uint8_t dydt		: 1;
	uint8_t mbz			: 1;
} ds3231_daydate_register;


// Alarm registers
/*
 * the registers for the date and time information generally contain the data in bcd. the 10's values may not be a full
 * four bits, some of the registers also have flags in them.
 */
typedef struct 
{
	uint8_t seconds			: 4;  // normal bcd
	uint8_t tensofseconds	: 3;
	uint8_t am1				: 1;
} ds3231_alarm_seconds_register;

typedef struct 
{
	uint8_t minutes			: 4;  // normal bcd
	uint8_t tensofminutes	: 3;
	uint8_t am2				: 1;
} ds3231_alarm_minutes_register;

typedef struct 
{
	uint8_t hour				: 4;	// this one is a little goofy
	uint8_t tensofhours			: 1;	//   when setting the time, if the twelvetwentyfour bit is set to 1, the clock is running
	uint8_t twentiesofhours		: 1;	//   running in 12 hour mode. in that case, twentiesofhours is 1 for pm and 0 for am.
	uint8_t twelvetwentyfour	: 1;	//   if twelvetwentyfour is set to 0, the clock is running in 24 hour mode. see the
	uint8_t am3					: 1;	//   note below
} ds3231_alarm_hours_register;

typedef struct 
{
	uint8_t day			: 4;
	uint8_t tensofdays	: 2;
	uint8_t dydt		: 1;
	uint8_t am4			: 1;
} ds3231_alarm_date_register;

typedef struct
{
	ds3231_alarm_seconds_register	seconds;		// 0x00
	ds3231_alarm_minutes_register	minutes;		// 0x01
	ds3231_alarm_hours_register		hours;		// 0x02
	ds3231_alarm_date_register		date;		// 0x03
} ds3231_alarm_registers;
 
typedef struct
{
	uint8_t	start_register;
	ds3231_alarm_registers	alarm;
} ds3231_alarm_transmission_packet;


/****************************************************************************************************************************/
// TWI functions

Twi *DS3231_TWI = TWI0;

uint32_t ds3231_twi_init(void)
{
	uint32_t status = twi_probe(DS3231_TWI, DS3231_ADDR);
	if( status != TWI_SUCCESS ) {
		printf("I2C probe FAILED: 0x%x\r\n", status);
	}
	
	return(status);
}

uint32_t ds3231_write(uint16_t addr, uint8_t *buf, uint16_t len)
{
	struct twi_packet p;
	p.addr[0] = 0;
	p.addr_length = 0;
	p.buffer = buf;
	p.length = len;
	p.chip = addr;

	uint32_t status = twi_master_write(DS3231_TWI, &p);

	return(status);	
}

uint32_t ds3231_read(uint16_t addr, uint8_t *buf, uint16_t len)
{
	struct twi_packet p;
	p.addr[0] = 0;
	p.addr_length = 0;
	p.buffer = buf;
	p.length = len;
	p.chip = addr;

	uint32_t status = twi_master_read(DS3231_TWI, &p);

	return(status);
}

/****************************************************************************************************************************/
//
// DS3231 functions

#define ASBYTE(b)       (*((uint8_t*) &(b)))

static bool	ds3231_configured = false;


uint32_t ds3231_read_register( uint8_t registerNumber, uint8_t *byte)
{
	uint32_t status = ~TWI_SUCCESS;

	/*
	 * Send the register number we are trying to read to the device. We do NOT want send a stop flag or the transaction
	 * will end up completed before we try to read the register data
	 */
	if ((status = ds3231_write(DS3231_ADDR, &registerNumber, sizeof(registerNumber))) != TWI_SUCCESS) {
		return status;
	}

	if ((status = ds3231_read(DS3231_ADDR, byte, sizeof(uint8_t))) != TWI_SUCCESS) {
		return status;
	}

	return status;
}
 
uint32_t ds3231_write_register( uint8_t registerNumber, uint8_t *byte)
{
	uint8_t data[2];
	uint32_t status = ~TWI_SUCCESS;

	/*
	 * Send the register number we are trying to write to the device followed by the data to be written.
	 */
	data[0] = registerNumber;
	data[1] = byte[0];

	status = ds3231_write(DS3231_ADDR, data, 2);

	return status;
}
 
 
uint32_t ds3231_init(void)
{
	ds3231_controlregister		controlreg;
	uint32_t			status = TWI_SUCCESS;
	ds3231_statusregister		statusreg;
	
	status = twi_probe(DS3231_TWI, DS3231_ADDR);
	if( status != TWI_SUCCESS ) {
		printf("ds3231_initialize(): I2C probe FAILED: 0x%x\r\n", status);
		return status;
	}
		
	/*
	 * set the control and status registers to values indicated as power on reset in the datasheet
	 * do this even if the time is considered valid
	 */
	controlreg.a1ie		= 0;		// alarm 1 enabled
	controlreg.a2ie		= 0;		// alarm 2 disabled
	controlreg.intcn	= 0;		// generate interrupt on alarm match
	controlreg.rs		= 0;		// square wave frequency 1Hz
	controlreg.conv		= 0;		// not requesting a temperature conversion
	controlreg.bbsqw	= 1;		// square wave output enabled
	controlreg.eoscnot	= 1;		// oscillator enabled
		 
	if ((status = ds3231_write_register(CONTROL_REGISTER, (uint8_t *)&controlreg)) != TWI_SUCCESS)
	{
		printf("ds3231_initialize(): ds3231_write_register(control_register) failed with status 0x%02x\r\n", status);
		return status;
	}
		 		 
	// writing status reg clears it
	statusreg.a1f		= 0;	// alarm 1 time match is an output
	statusreg.a2f		= 0;	// alarm 2 time match is an output
	statusreg.bsy		= 0;	// busy flag is an output
	statusreg.en32khz	= 1;	// 32khz output pin enabled
	statusreg.mbz		= 0;	// must be zero
	statusreg.osf		= 0;	// clear oscillator stop flag

	if ((status = ds3231_write_register(STATUS_REGISTER, (uint8_t *)&statusreg)) != TWI_SUCCESS)
	{
		printf("ds3231_initialize(): ds3231_write_register(status_register) failed with status 0x%02x\r\n",status);
		return status;
	}

	uint8_t reg;
	if ((status = ds3231_read_register(STATUS_REGISTER, &reg)) != TWI_SUCCESS)
	{
		printf("ds3231_initialize(): ds3231_read_register(status_register) failed with status 0x%02x\r\n",status);
		return status;
	}
	//printf("ds3231 status reg: 0x%X\r\n", reg);
	
	ds3231_configured = true;	
	 
	return status;
}
 
 /****************************************************************************************************************************
 **
 ** rtc_get_datetime()
 **/
uint8_t  ds3231_baseyear = 20;

uint32_t ds3231_get_datetime( ds3231_datetime*  dt)
{
	ds3231_datetimeregisters datetime;
	uint8_t secondsregister = SECONDS_REGISTER;
	uint32_t status = ~TWI_SUCCESS;
	
	/* assume failure */
	dt->year		= 0;
	dt->month		= 0;
	dt->day			= 0;
	dt->hour		= 0;
	dt->minute		= 0;
	dt->second		= 0;
	dt->century		= 20;
	
	ds3231_statusregister reg;
	if ((status = ds3231_read_register(STATUS_REGISTER, (uint8_t *)&reg)) != TWI_SUCCESS)
	{
		printf("ds3231_get_datetime(): ds3231_read_register(status_register) failed with status 0x%02x\r\n",status);
		return status;
	}
	
	// check if RTC osc has stopped for any reason
	if( reg.osf == 1 ) {
		status = TWI_INVALID_ARGUMENT;
		printf("ds3231_get_datetime(): status_register osf bit set, invalid date: 0x%02x\r\n", reg);
		return status;
	}
	 
	/* read the time from the device */
	if ((status = ds3231_write(DS3231_ADDR, &secondsregister, sizeof(secondsregister))) != TWI_SUCCESS)
	{
		printf("ds3231_get_datetime() selecting the seconds register failed with status 0x%02x\r\n", status);
		status = TWI_INVALID_ARGUMENT;
		return status;
	}
	
	memset(&datetime, 0, sizeof(datetime));
	
	if ((status = ds3231_read(DS3231_ADDR, (uint8_t*) &datetime, sizeof(datetime))) != TWI_SUCCESS)
	{
		printf("ds3231_get_datetime() reading the date/time data failed with status 0x%02x\r\n", status);
		status = TWI_INVALID_ARGUMENT;
		return status;
	}
	
	/* convert the response to a simple date time structure */
	
	dt->second = (datetime.seconds.tensofseconds * 10) + datetime.seconds.seconds;
	
	dt->minute = (datetime.minutes.tensofminutes * 10) + datetime.minutes.minutes;
	
	if (datetime.hours.twelvetwentyfour == 0)
	{
		/* the clock is running in 24 hour mode */
		dt->hour = datetime.hours.hour;
		if (datetime.hours.tensofhours != 0)
		{
			dt->hour += 10;
		}
		else if (datetime.hours.twentiesofhours != 0)
		{
			dt->hour += 20;
		}
	}
	else
	{
		/* twelve hour mode */
		dt->hour = (datetime.hours.tensofhours * 10) + datetime.hours.hour;
		if (datetime.hours.twentiesofhours != 0)
		{
			/* current time is after noon */
			dt->hour += 12;
		}
	}
	
	dt->day = (datetime.date.tensofdays * 10) + datetime.date.day;
	
	dt->month = (datetime.monthcentury.tensofmonths * 10) + datetime.monthcentury.month;
	
	dt->year = (datetime.year.tensofyears * 10) + datetime.year.year;
	//dt->year = (ds3231_baseyear * 100) + (datetime.year.tensofyears * 10) + datetime.year.year;
	
	//if (datetime.monthcentury.century != 0)
	//{
	//	dt->year += 100;
	//}
	
	status = TWI_SUCCESS;
	
	return status;
}

/****************************************************************************************************************************
 **
 ** ds3231_set_data_time()
 **/

uint32_t ds3231_set_datetime( ds3231_datetime*  dt)
{
	//ds3231_datetimeregisters datetime;
	uint32_t status = ~TWI_SUCCESS;
	//uint8_t baseYear;
	ds3231_datetimetransmissionpacket packet;
	uint16_t yearofcentury;

	/*
	 * Confirm that the date/time information that we were supplied is valid
	 */
	if (!ds3231_datetime_valid(dt))
	{
		printf("ds3231_set_datetime() has been called with an invalid date/time\r\n");
		status = TWI_INVALID_ARGUMENT;
		goto EXIT;
	}

	/*
	 * Convert the supplied date/time to the format needed by the RTC
	 *
	 * -- For sending the date/time we need to select the first register that we will be writing to
	 */
	packet.startregister = SECONDS_REGISTER;

	/*
	 * -- Seconds: Convert to BCD
	 */
	packet.dt.seconds.seconds		= dt->second % 10;
	packet.dt.seconds.tensofseconds	= dt->second / 10;
	packet.dt.seconds.mbz			= 0;

	/*
	 * -- Minutes: Convert to BCD
	 */
	packet.dt.minutes.minutes		= dt->minute % 10;
	packet.dt.minutes.tensofminutes	= dt->minute / 10;
	packet.dt.minutes.mbz			= 0;

	/*
	 * -- Hours: Convert to 24 hour format as described in the definition of HoursRegister
	 */
	packet.dt.hours.hour = dt->hour % 10;

	if ((dt->hour >= 10)
	 && (dt->hour <= 19))
	{
		packet.dt.hours.tensofhours     = 1;
		packet.dt.hours.twentiesofhours = 0;
	}
	else if ((dt->hour >= 20)
		  && (dt->hour <= 23))
	{
		packet.dt.hours.tensofhours     = 0;
		packet.dt.hours.twentiesofhours = 1;
	}
	else
	{
		packet.dt.hours.tensofhours     = 0;
		packet.dt.hours.twentiesofhours = 0;
	}

	packet.dt.hours.twelvetwentyfour = 0;   // select 24 hour mode
	packet.dt.hours.mbz = 0;

	/*
	 * -- DayOfWeek: Always assume that this date/time is the start of the week
	 */
	packet.dt.day.day = 1;
	packet.dt.day.mbz = 0;

	/*
	 * -- day of month: convert to bcd
	 */
	packet.dt.date.day			= dt->day % 10;
	packet.dt.date.tensofdays	= dt->day / 10;
	packet.dt.date.mbz			= 0;

	/*
	 * -- month: convert to bcd. note that we will always assume that this year is the base year so we don't have to worry
	 *           about the century flag
	 */
	packet.dt.monthcentury.month		= dt->month % 10;
	packet.dt.monthcentury.tensofmonths	= dt->month / 10;
	packet.dt.monthcentury.mbz			= 0;
	packet.dt.monthcentury.century		= 0;

	/*
	 * -- Year: All years are allowed, but we need to keep 0-99 for the RTC and save off the century in the signature block
	 */
	yearofcentury	= dt->year % 100;
	ds3231_baseyear	= dt->year / 100;

	packet.dt.year.year			= yearofcentury % 10;
	packet.dt.year.tensofyears	= yearofcentury / 10;

	/*
	 * Write the date/time to the device
	 */
	if ((status = ds3231_write(DS3231_ADDR, (uint8_t*) &packet, sizeof(packet))) != TWI_SUCCESS)
	{
		printf("ds3231_set_datetime() storing the date/time to the device failed with status 0x%02x\r\n",status);
		goto EXIT; 
	}


EXIT:
	return status;
}



/****************************************************************************************************************************
 **
 **/

static uint16_t daysPerMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

bool ds3231_datetime_valid(const ds3231_datetime *dt)
{
	uint16_t	days;
	bool		simpleDateTimeValid = false;

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
	if (dt->second > 59)
	{
		goto EXIT;
	}

	/*
	 * -- Minutes are 0 to 59
	 */
	if (dt->minute > 59)
	{
		goto EXIT;
	}

	/*
	 * Hours are 0 to 23
	 */
	if (dt->hour > 23)
	{
		goto EXIT;
	}

	/*
	 * Months are 1 to 12
	 */
	if ((dt->month == 0)
	 || (dt->month > 12))
	{
		goto EXIT;
	}

	/*
	 * The day of the month is based on the number of days in the month and whether or not the year is a leap year
	 */
	if (dt->month == 2)						// February
	{
		if ((dt->year % 400) == 0)			// Years which are multiples of 400 are leap years
		{
			days = 29;
		}
		else if ((dt->year % 100) == 0)		// Years which are multiples of 100 (but not 400) are not leap years
		{
			days = 28;
		}
		else if ((dt->year % 4) == 0)		// "Normal" leap years
		{
			days = 29;
		}
		else										// The only case left is non leap years
		{
			days = 28;
		}

		if ((dt->day == 0)
		 || (dt->day > days))
		{
			goto EXIT;
		}
	}
	else if ((dt->day == 0)					// All months other than February can use the lookup table
		  || (dt->day > daysPerMonth[dt->month - 1]))
	{
		goto EXIT;
	}

	// We don't really care about the year value, but we need to be consistent with the signature
	if (((dt->year / 100) < BASE_YEAR_2000)
	 || ((dt->year / 100) > BASE_YEAR_2500))
	{
		goto EXIT;
	}

	/*
	 * All checks have passed
	 */
	simpleDateTimeValid = true;

EXIT:
	return simpleDateTimeValid;
}

uint32_t ds3231_set_alarm( ds3231_datetime *dt )
{
	ds3231_controlregister	controlreg;
	ds3231_statusregister	statusreg;
	ds3231_alarm_transmission_packet packet;
	uint32_t	status = ~TWI_SUCCESS;

	// set alarm registers
	packet.start_register = ALARM1_SECONDS_REGISTER;
	
	packet.alarm.seconds.seconds		= dt->second % 10;
	packet.alarm.seconds.tensofseconds	= dt->second / 10;

	// Minutes: Convert to BCD
	packet.alarm.minutes.minutes		= dt->minute % 10;
	packet.alarm.minutes.tensofminutes	= dt->minute / 10;

	// Hours: Convert to 24 hour format as described in the definition of HoursRegister
	packet.alarm.hours.hour = dt->hour % 10;

	if ((dt->hour >= 10) && (dt->hour <= 19))
	{
		packet.alarm.hours.tensofhours     = 1;
		packet.alarm.hours.twentiesofhours = 0;
	}
	else if ((dt->hour >= 20) && (dt->hour <= 23))
	{
		packet.alarm.hours.tensofhours     = 0;
		packet.alarm.hours.twentiesofhours = 1;
	}
	else
	{
		packet.alarm.hours.tensofhours     = 0;
		packet.alarm.hours.twentiesofhours = 0;
	}

	packet.alarm.hours.twelvetwentyfour = 0;   // select 24 hour mode

	// day of month
	packet.alarm.date.day			= dt->day % 10;
	packet.alarm.date.tensofdays	= dt->day / 10;
	packet.alarm.date.dydt = 0;  // use date of month: 1-31

	packet.alarm.seconds.am1 = 0;  // A1M1
	packet.alarm.minutes.am2 = 0;  // A1M2
	packet.alarm.hours.am3 = 0;  // A1M3
	packet.alarm.date.am4 = 0;  // A1M4

	/*
	 * Write the alarm packet to the device
	 */
	if ((status = ds3231_write(DS3231_ADDR, (uint8_t*) &packet, sizeof(packet))) != TWI_SUCCESS)
	{
		printf("ds3231_set_alarm() setting alarm failed with status 0x%02x\r\n",status);
		return(status);
	}
	
	/*
	 * Read the Control register
	 */
//	if ((status = ds3231_read_register(CONTROL_REGISTER, &controlreg)) != TWI_SUCCESS)
//	{
//		printf("ds3231_set_alarm(): reading control register failed with status 0x%02x\r\n", status);
//		return status;
//	}
	
	/*
	 * Write the Control register
	 */
	/*
	 * set the control and status registers to values indicated as power on reset in the datasheet
	 * do this even if the time is considered valid
	 */
	controlreg.a1ie		= 1;		// alarm 1 enabled
	controlreg.a2ie		= 0;		// alarm 2 disabled
	controlreg.intcn	= 1;		// generate interrupt on alarm match
	controlreg.rs		= 0;		// square wave frequency 1khz, if unused
	controlreg.conv		= 0;		// not requesting a temperature conversion
	controlreg.bbsqw	= 0;		// square wave output disabled
	controlreg.eoscnot	= 1;		// oscillator enabled
		 
	if ((status = ds3231_write_register(CONTROL_REGISTER, (uint8_t *)&controlreg)) != TWI_SUCCESS)
	{
		printf("ds3231_configure(): ds3231_write_register(control_register) failed with status 0x%02x\r\n", status);
		return status;
	}
		 
	// writing status reg clears it
	statusreg.a1f		= 0;	// alarm 1 time match is an output
	statusreg.a2f		= 0;	// alarm 2 time match is an output
	statusreg.bsy		= 0;	// busy flag is an output
	statusreg.en32khz	= 1;		// 32khz output pin disabled
	statusreg.mbz		= 0;			// must be zero
	statusreg.osf		= 0;		// clear oscillator stop flag
		 
	if ((status = ds3231_write_register(STATUS_REGISTER, (uint8_t *)&statusreg)) != TWI_SUCCESS)
	{
		printf("ds3231_configure(): ds3231_write_register(status_register) failed with status 0x%02x\r\n",status);
		return status;
	}

		 
	return status;
}

/****************************************************************************************************************************/
//
// Epoch functions


// 01 Jan 2000 00:00:00 GMT
#define DS3231_EPOCH_OFFSET 946684800  
 
static unsigned short LEAP_DAYS[4][12] =
{
	{   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
	{ 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
	{ 731, 762, 790, 821, 851, 882, 912, 943, 974,1004,1035,1065},
	{1096,1127,1155,1186,1216,1247,1277,1308,1339,1369,1400,1430},
};

//
// ???? Need to verify that day and month start at zero instead of 1
//
unsigned int ds3231_datetime_to_epoch(ds3231_datetime *dt)
{
	unsigned int second = (unsigned int)(dt->second);  // 0-59
	unsigned int minute = (unsigned int)(dt->minute);  // 0-59
	unsigned int hour   = (unsigned int)(dt->hour);    // 0-23
	unsigned int day    = (unsigned int)(dt->day-1);   // 0-30
	unsigned int month  = (unsigned int)(dt->month-1); // 0-11
	unsigned int year   = (unsigned int)(dt->year - 2000);    // 0-99
	return (((year/4*(365*4+1)+LEAP_DAYS[year%4][month]+day)*24+hour)*60+minute)*60+second + DS3231_EPOCH_OFFSET;
}


void ds3231_epoch_to_datetime(ds3231_datetime *dt, unsigned int epoch)
{
	epoch = epoch - DS3231_EPOCH_OFFSET;
	
	dt->second = (uint16_t)(epoch%60); epoch /= 60;
	dt->minute = (uint16_t)(epoch%60); epoch /= 60;
	dt->hour   = (uint16_t)(epoch%24); epoch /= 24;

	unsigned int years = (epoch/(365*4+1)*4);
	epoch %= 365*4+1;

	unsigned int year;
	for (year=3; year>0; year--)
	{
		if (epoch >= LEAP_DAYS[year][0])
		break;
	}

	unsigned int month;
	for (month=11; month>0; month--)
	{
		if (epoch >= LEAP_DAYS[year][month])
		break;
	}

	dt->year  = (uint16_t)(years+year + 2000);
	dt->month = (uint16_t)(month+1);
	dt->day   = (uint16_t)(epoch-LEAP_DAYS[year][month]+1);
}
