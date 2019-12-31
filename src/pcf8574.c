/*
 * pcf8574.c
 *
 * The PCF8574 device is an 8-bit I/O expander for the two-line bidirectional bus (I2C) is 
 designed for 2.5-V to 5.5-
 * V VCC operation. It provides general-purpose remote I/O expansion for most micro-controller families via the I2C
 interface (serial clock, SCL, and serial data, SDA, pins).
 
 *
 * Created: 12/19/2019 7:10:33 PM
 *  Author: Chris
 */ 

#include	<asf.h>
#include	<assert.h>
#include	<string.h>

#include "i2c.h"
#include "pcf8574.h"

#define PCF8574_ADDR  (0x40 >> 1)

Twi *PCF8574_TWI = TWI0;

/*
I2C communication with this device is initiated by a master sending a start condition, a high-to-low transition on
the SDA I/O while the SCL input is high. After the start condition, the device address byte is sent, mostsignificant
bit (MSB) first, including the data direction bit (R/W). This device does not respond to the general call
address. After receiving the valid address byte, this device responds with an acknowledge, a low on the SDA I/O
during the high of the acknowledge-related clock pulse. The address inputs (A0–A2) of the slave device must not
be changed between the start and the stop conditions.
The data byte follows the address acknowledge. If the R/W bit is high, the data from this device are the values
read from the P port. If the R/W bit is low, the data are from the master, to be output to the P port. The data byte
is followed by an acknowledge sent from this device. If other data bytes are sent from the master, following the
acknowledge, they are ignored by this device. Data are output only if complete bytes are received and
acknowledged. The output data will be valid at time, tpv, after the low-to-high transition of SCL and during the
clock cycle for the acknowledge.
A stop condition, which is a low-to-high transition on the SDA I/O while the SCL input is high, is sent by the
master.
*/
int pcf8574_write(uint8_t byte)
{
	int status = ~TWI_SUCCESS;
	status = i2c_write(PCF8574_TWI, PCF8574_ADDR, &byte, 1);
	if ( status != TWI_SUCCESS) {
		printf("pcf8574: error writing (0x%02x)\r\n", status);
	}
	return status;
}

/*
*/
int pcf8574_read(uint8_t *byte)
{
	int status = ~TWI_SUCCESS;
	status = i2c_read(PCF8574_TWI, PCF8574_ADDR, byte, 1);
	if ( status != TWI_SUCCESS) {
		printf("pcf8574: error reading (0x%02x)\r\n", status);
	}
	return status;
}

