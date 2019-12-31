/*
 * i2c.c
 *
 * Created: 12/19/2019 4:01:24 PM
 *  Author: Chris
 */ 

#include <twi.h>
#include <board.h>
#include <gpio.h>
#include <ioport.h>
#include "i2c.h"

// 
// I2C Configuration
//
int i2c_init(Twi *twi, uint32_t speed)
{
	uint32_t status = TWI_INVALID_ARGUMENT;

	// configure TWI0 pins
	if(twi == TWI0) {
		pio_configure_pin(PIN_TWI0_DATA_GPIO, PIN_TWI0_DATA_FLAGS);
		pio_configure_pin(PIN_TWI0_CLK_GPIO, PIN_TWI0_CLK_FLAGS);
			
		//Allow TWI to control PA3 and PA4
		pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA3A_TWD0 | PIO_PA4A_TWCK0);

		if(pmc_enable_periph_clk(ID_TWI0) != TWI_SUCCESS) {
			printf("I2C master PERIPHERAL CLOCK FAILED\r\n");
		}
		//printf("I2C master PERIPHERAL CLOCK EANABlED\r\n");
	}

	// configure TWI1 pins
	if(twi == TWI1) {
		//Set PB4 and PB5 as GPIO, not JTAG pins
		REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
		REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO5;

		// configure TWI1 pins
		pio_configure_pin(PIN_TWI1_DATA_GPIO, PIN_TWI1_DATA_FLAGS);
		pio_configure_pin(PIN_TWI1_CLK_GPIO, PIN_TWI1_CLK_FLAGS);
	
		//Allow TWI to control PB4 and PB5
		pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB4A_TWD1 | PIO_PB5A_TWCK1);

		if(pmc_enable_periph_clk(ID_TWI1) != TWI_SUCCESS) {
			printf("I2C master PERIPHERAL CLOCK FAILED\r\n");
		}
		//printf("I2C master PERIPHERAL CLOCK EANABlED\r\n");
	}
	
	twi_options_t opt;
	opt.master_clk = sysclk_get_main_hz();
	opt.speed = speed; //100000UL;
	opt.smbus = 0;

	status = twi_master_init(twi, &opt);
	if( status != TWI_SUCCESS ) {
		printf("I2C master init FAILED: 0x%x\r\n", status);
	}
	//printf("I2C master init: 0x%x\r\n", status);

	return (int)status;
}


int i2c_write(Twi *twi, uint16_t addr, uint8_t *buf, uint16_t len)
{
	struct twi_packet p;
	p.addr[0] = 0;
	p.addr_length = 0;
	p.buffer = buf;
	p.length = len;
	p.chip = addr;

	uint32_t status = twi_master_write(twi, &p);

	return (int)status;
}

int i2c_read(Twi *twi, uint16_t addr, uint8_t *buf, uint16_t len)
{
	struct twi_packet p;
	p.addr[0] = 0;
	p.addr_length = 0;
	p.buffer = buf;
	p.length = len;
	p.chip = addr;

	uint32_t  status = twi_master_read(twi, &p);

	return (int)status;
}

