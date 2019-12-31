/*
 * ina260.h
 *
 * Created: 12/19/2019 5:24:03 PM
 *  Author: Chris
 */ 


#ifndef INA260_H_
#define INA260_H_

int ina260_write_register(uint8_t regnum, uint8_t *reg);
int ina260_read_register( uint8_t regnum, uint8_t *reg);

int ina260_init(void);
int ina260_read_power(int32_t *amps, uint32_t *volts);


#endif /* INA260_H_ */