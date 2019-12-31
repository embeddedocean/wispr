/*
 * ina260.c
 *
 * Created: 12/19/2019 4:44:02 PM
 *  Author: Chris
 */ 

#include	<asf.h>
#include	<assert.h>
#include	<string.h>

#include "ina260.h"
#include "i2c.h"

#define INA260_ADDR  (0x80 >> 1)

Twi *INA260_TWI = TWI0;

// INA260 Control Registers
typedef struct 
{
	uint8_t mode	: 3;	// Operating more
	uint8_t ishct	: 3;	// Shunt Current Conversion Time
	uint8_t vbusct	: 3;	// Bus Voltage Conversion Time
	uint8_t avg	: 3;	// Averaging Mode
	uint8_t mbz	: 3;	// 110
	uint8_t rst	: 1;	// Reset Bit
} ina260_control_register;

/*
Writing to a register begins with the first byte transmitted by the master. This byte is the slave address, with the
R/W bit low. The device then acknowledges receipt of a valid address. The next byte transmitted by the master is
the address of the register which data is written to. This register address value updates the register pointer to the
desired register. The next two bytes are written to the register addressed by the register pointer. The device
acknowledges receipt of each data byte. The master may terminate data transfer by generating a start or stop
condition.
*/
int ina260_write_register(uint8_t regnum, uint8_t *reg)
{
	uint8_t data[3];
	int status = ~TWI_SUCCESS;

	/*
	 * Send the register number we are trying to write to the device followed by the data to be written.
	 */
	data[0] = regnum;
	data[1] = reg[0];
	data[2] = reg[1];

	status = i2c_write(INA260_TWI, INA260_ADDR, data, 3);

	return status;
}

/*
When reading from the device, the last value stored in the register pointer by a write operation determines which
register is read during a read operation. To change the register pointer for a read operation, a new value must be
written to the register pointer. This write is accomplished by issuing a slave address byte with the R/W bit low,
followed by the register pointer byte. No additional data are required. The master then generates a start condition
and sends the slave address byte with the R/W bit high to initiate the read command. The next byte is
transmitted by the slave and is the most significant byte of the register indicated by the register pointer. This byte
is followed by an Acknowledge from the master; then the slave transmits the least significant byte. The master
acknowledges receipt of the data byte. The master may terminate data transfer by generating a Not-
Acknowledge after receiving any data byte, or generating a start or stop condition. If repeated reads from the
same register are desired, it is not necessary to continually send the register pointer bytes; the device retains the
register pointer value until it is changed by the next write operation.
*/
int ina260_read_register( uint8_t regnum, uint8_t *reg)
{
	int status = ~TWI_SUCCESS;

	/*
	 * Send the register number we are trying to read to the device. We do NOT want send a stop flag or the transaction
	 * will end up completed before we try to read the register data
	 */
	if((status = i2c_write(INA260_TWI, INA260_ADDR, &regnum, 1)) != TWI_SUCCESS) {
		return status;
	}

	if((status = i2c_read(INA260_TWI, INA260_ADDR, reg, 2)) != TWI_SUCCESS) {
		return status;
	}
	//printf("ina260_read_register: 0x%02x 0x%02x\r\n", reg[0], reg[1]);

	return status;
}


int ina260_init()
{
	int status = ~TWI_SUCCESS;
	
	ina260_control_register ctrl;
	
	ctrl.mode = 7;		// Shunt Current and Bus Voltage, Continuous
	ctrl.ishct = 7;		// conversion time = 8.244 ms
	ctrl.vbusct = 7;	// conversion time = 8.244 ms
	ctrl.avg = 7;		// number of averages = 1024
	ctrl.mbz = 6;		// constant
	ctrl.rst = 1;		// reset

	// control is register 0
	status = ina260_write_register(0, (uint8_t *)&ctrl);
	if ( status != TWI_SUCCESS) {
		printf("ina260_configure: error writing control_register (0x%02x)\r\n", status);
	}

	ctrl.rst = 0;		// reset
	status = ina260_write_register(0, (uint8_t *)&ctrl);
	if ( status != TWI_SUCCESS) {
		printf("ina260_configure: error writing control_register (0x%02x)\r\n", status);
	}
	
	return status;
}

int ina260_read_power(int32_t *amps, uint32_t *volts)
{
	int  status = ~TWI_SUCCESS;
	uint8_t reg[2];
	
	// current is register 1
	int16_t mA;
	status = ina260_read_register(1, reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_read_power: error reading current register (0x%02x)\r\n", status);
	}
	// swap the bytes in the LSB regs to convert to an int16
	mA = ((int16_t)reg[0] << 8) | ((int16_t)reg[1] << 0);
	*amps = (int32_t)(1.25f * (float)mA);
	
	// voltage is register 2
	uint16_t mV;
	status = ina260_read_register(2, reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_read_power: error reading  register (0x%02x)\r\n", status);
	}
	// swap the bytes in the LSB regs to convert to an uint16
	mV = ((uint16_t)reg[0] << 8) | ((uint16_t)reg[1] << 0);
	*volts = (uint32_t)(1.25f * (float)mV);
	
	return status;
}

 