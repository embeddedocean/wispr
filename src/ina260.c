/*
 * ina260.c
 *
 * Created: 12/19/2019 4:44:02 PM
 *  Author: Chris
 */ 

#include	<asf.h>
#include	<assert.h>
#include	<stdio.h>
#include	<string.h>

#include "ina260.h"
#include "i2c.h"
#include "com.h"

#define INA260_ADDR  (0x80 >> 1)

#define INA260_CNRF 0x08

Twi *INA260_TWI = TWI0;

static uint16_t ina260_config = 0;
static uint8_t ina260_send_com_msg = 0;

float32_t ina260_V = 0.0; // Volts
float32_t ina260_mA = 0.0;  // mAmps
float32_t ina260_mWh = 0.0;  // mWatt Hours
float32_t ina260_mAh = 0.0;  // mAmp Hours

float32_t ina260_dt = 0.0011; // sampling interval in seconds

/*
Writing to a register begins with the first byte transmitted by the master. This byte is the slave address, with the
R/W bit low. The device then acknowledges receipt of a valid address. The next byte transmitted by the master is
the address of the register which data is written to. This register address value updates the register pointer to the
desired register. The next two bytes are written to the register addressed by the register pointer. The device
acknowledges receipt of each data byte. The master may terminate data transfer by generating a start or stop
condition.
*/
static int ina260_write_register(uint8_t regnum, uint8_t *reg)
{
	uint8_t data[3];
	int status = ~TWI_SUCCESS;

	/*
	 * Send the register number we are trying to write to the device followed by the data to be written.
	 */
	data[0] = regnum;
	data[1] = reg[1];
	data[2] = reg[0];

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
static int ina260_read_register( uint8_t regnum, uint8_t *reg)
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
	//printf("ina260_read_register %d: 0x%02x 0x%02x\r\n", regnum, reg[0], reg[1]);

	return status;
}

static void ina260_alarm_handler(uint32_t ul_id, uint32_t ul_mask)
{
	if (ID_PIOA == ul_id && PIO_PA5 == ul_mask) {

		ina260_read(&ina260_mA, &ina260_V, 2);

		// cummulative mAh and mWh
		ina260_mAh += (ina260_mA * ina260_dt * 0.0002778); //  mAh
		ina260_mWh += (ina260_V * ina260_mA * ina260_dt * 0.0002778); //  mWh
				
		//printf("INA260 Alarm: I = %f mA, V = %f V\r\n", ina260_mA, ina260_V);

		if(ina260_send_com_msg) {
			char str[64];
			sprintf(str, "PWR,%.2f,%.2f,%.2f", ina260_V, ina260_mA, ina260_mWh);
			//sprintf(str, "PWR,%.2f,%.2f,%.2f", ina260_mA, ina260_V, ina260_mWh);
			com_write_msg(BOARD_COM_PORT, str);			
		}

	}
}

int ina260_init(uint16_t config, uint16_t alarm, uint8_t send_com_msg)
{
 	int status = ~TWI_SUCCESS;
	uint16_t reg = 0;
	
	if( alarm > 0) {
		
		// setup pin used for alarm
		ioport_set_pin_dir(PIN_INA260_ALARM, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(PIN_INA260_ALARM, IOPORT_MODE_PULLUP);

		/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
		pio_set_debounce_filter(PIN_INA260_ALARM_PIO, PIN_INA260_ALARM_MASK, 10);
	
		// Initialize PIO interrupt handler for alarm pin with falling edge or low detection
		uint32_t attr = (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE ); //PIO_IT_LOW_LEVEL);
		pio_handler_set(PIN_INA260_ALARM_PIO, PIN_INA260_ALARM_ID, PIN_INA260_ALARM_MASK, attr, ina260_alarm_handler);

		// Enable PIOA controller IRQs
		//NVIC_DisableIRQ(PIOA_IRQn);
		//NVIC_ClearPendingIRQ(PIOA_IRQn);
		NVIC_SetPriority(PIOA_IRQn, 2);
		//NVIC_EnableIRQ((IRQn_Type)PIN_INA260_ALARM_ID);
		NVIC_EnableIRQ(PIOA_IRQn);		
		
		// Enable line interrupts
		pio_enable_interrupt(PIN_INA260_ALARM_PIO, PIN_INA260_ALARM_MASK);
		
	}
	
	ina260_send_com_msg = send_com_msg;

	// reset - control is register 0
	reg = 0x8000;
	status = ina260_write_register(0, (uint8_t *)&reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_init: error writing control_register (0x%02x)\r\n", status);
	}
	delay_ms(1);

	// read defaults config regs values
	status = ina260_read_register(0, (uint8_t *)&reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_init: error writing control_register (0x%02x)\r\n", status);
	}

	// set configure register bits
	reg = config;
	
	// save config because the config needs to be re-written after each conversion in triggered mode
	ina260_config = reg; 

	// control is register 0
	status = ina260_write_register(0, (uint8_t *)&reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_init: error writing control_register (0x%02x)\r\n", status);
	}

	// enable alarms
	reg = alarm; 
	
	// mask/enable is register 6
	status = ina260_write_register(6, (uint8_t *)&reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_init: error writing enable register (0x%02x)\r\n", status);
	}

	// initialize local variables
	ina260_mA = 0;
	ina260_V = 0;
	ina260_mWh = 0;
	ina260_mAh = 0;
	ina260_dt = 0.0011;

	uint16_t ct = (config & 0x01F8);
	if(ct == INA260_CONFIG_CT_140us)  ina260_dt = 0.000140;
	else if(ct == INA260_CONFIG_CT_204us)  ina260_dt = 0.000204;
	else if(ct == INA260_CONFIG_CT_332us)  ina260_dt = 0.000332;
	else if(ct == INA260_CONFIG_CT_588us)  ina260_dt = 0.000588;
	else if(ct == INA260_CONFIG_CT_1100us) ina260_dt = 0.0011;
	else if(ct == INA260_CONFIG_CT_2116us) ina260_dt = 0.002116;
	else if(ct == INA260_CONFIG_CT_4156us) ina260_dt = 0.004156;
	else if(ct == INA260_CONFIG_CT_8244us) ina260_dt = 0.008244;
	
	uint16_t avg = (config & 0x0E00);
	if(avg == INA260_CONFIG_AVG_1)  ina260_dt *= 1.0;
	else if(avg == INA260_CONFIG_AVG_4) ina260_dt *= 4.0;
	else if(avg == INA260_CONFIG_AVG_16) ina260_dt *= 16.0;
	else if(avg == INA260_CONFIG_AVG_64) ina260_dt *= 64.0;
	else if(avg == INA260_CONFIG_AVG_128) ina260_dt *= 128.0;
	else if(avg == INA260_CONFIG_AVG_256) ina260_dt *= 256.0;
	else if(avg == INA260_CONFIG_AVG_512) ina260_dt *= 512.0;
	else if(avg == INA260_CONFIG_AVG_1024) ina260_dt *= 1024.0;

	//printf("ina260_init: config register 0x%04x\r\n", config);
	//printf("ina260_init: sampling interval %f\r\n", ina260_dt);
	
	return status;
}


int ina260_stop()
{
	int status = ~TWI_SUCCESS;
	uint16_t reg = 0;  // shutdown
		
	// disable conversion
	reg = 0; 
	
	// enable is register 6
	status = ina260_write_register(6, (uint8_t *)&reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_init: error writing enable register (0x%02x)\r\n", status);
	}
	
	// control is register 0
	status = ina260_write_register(0, (uint8_t *)&reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_stop: error writing control_register (0x%02x)\r\n", status);
	}

	// disable line interrupts
	pio_disable_interrupt(PIN_INA260_ALARM_PIO, PIN_INA260_ALARM_MASK);

	return status;
}

int ina260_read(float32_t *mA, float32_t *V, uint32_t timeout)
{
	int  status = ~TWI_SUCCESS;
	uint8_t reg[2];
	int16_t wait = 1;

	// read mask/enable register and wait until a new conversion is ready
	if(timeout == 0) wait = 1; // just read the reg once
	else wait = timeout;
	
	while( wait > 0 ) {	
		// read Mask/Enable Register (06h)
		status = ina260_read_register(6, (uint8_t *)reg);
		if ( status != TWI_SUCCESS) {
			printf("ina260_read: error reading MAsk/Enable register (0x%02x)\r\n", status);
		}
		if( reg[1] == INA260_CNRF  ) { // check if CVRF bit is set 
			//printf("ina260_read_power: cnrf at %d\r\n", wait);
			break;
		} 
		else wait--;
		delay_ms(10); // slow things down
	}
	
	if(wait == 0) {
		// return the last measurements
		*mA = ina260_mA;
		*V = ina260_V;
		//printf("ina260_read: timeout\r\n");
		return(0);
	}
	
	// current is register 1
	int16_t iv;
	status = ina260_read_register(1, reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_read: error reading current register (0x%02x)\r\n", status);
	}
	// swap the bytes in the LSB regs to convert to an int16
	iv = ((int16_t)reg[0] << 8) | ((int16_t)reg[1] << 0);
	*mA = (1.25f * (float32_t)iv);
	
	// voltage is register 2
	uint16_t uv;
	status = ina260_read_register(2, reg);
	if ( status != TWI_SUCCESS) {
		printf("ina260_read: error reading  register (0x%02x)\r\n", status);
	}
	// swap the bytes in the LSB regs to convert to an uint16
	uv = ((uint16_t)reg[0] << 8) | ((uint16_t)reg[1] << 0);
	*V = 0.001 * (float32_t)(1.25f * (float32_t)uv);

	// control is register 0
	status = ina260_write_register(0, (uint8_t *)&ina260_config);
	if ( status != TWI_SUCCESS) {
		printf("ina260_read: error rewriting control_register (0x%02x)\r\n", status);
	}
	
	return(1);
}

 