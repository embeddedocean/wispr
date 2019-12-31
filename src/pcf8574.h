/*
 * pcf8574.h
 *
 * Created: 12/20/2019 1:22:20 PM
 *  Author: Chris
 */ 


#ifndef PCF8574_H_
#define PCF8574_H_

#define GP0 0x01
#define GP1 0x02
#define GP2 0x04
#define GP3 0x08
#define GP4 0x10
#define GP5 0x20
#define GP6 0x40
#define GP7 0x80
 
int pcf8574_write(uint8_t byte);
int pcf8574_read(uint8_t *byte);


#endif /* PCF8574_H_ */