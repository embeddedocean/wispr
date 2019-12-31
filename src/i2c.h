/*
 * i2c.h
 *
 * Created: 12/19/2019 4:02:00 PM
 *  Author: Chris
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <status_codes.h>

int i2c_init(Twi *twi, uint32_t speed);
int i2c_write(Twi *twi, uint16_t addr, uint8_t *buf, uint16_t len);
int i2c_read(Twi *twi, uint16_t addr, uint8_t *buf, uint16_t len);


#endif /* I2C_H_ */