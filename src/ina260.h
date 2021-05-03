/*
 * ina260.h
 *
 * Created: 12/19/2019 5:24:03 PM
 *  Author: EOS Chris
 */ 


#ifndef INA260_H_
#define INA260_H_

#include <arm_math.h>

// Alarm bits in the Mask/Enable Register
#define INA260_ALARM_NONE 0 // disable all alarms
#define INA260_ALARM_CONVERSION_READY (0x0400)
#define INA260_ALARM_UNDER_VOLTAGE    (0x1000)
#define INA260_ALARM_OVER_VOLTAGE     (0x2000)

// Operating Mode - Configuration register bits
// Selects continuous, triggered, or power-down mode of operation
#define INA260_CONFIG_MODE_SHUTDOWN   0
#define INA260_CONFIG_MODE_CONTINUOUS (0x0007)
#define INA260_CONFIG_MODE_TRIGGERED  (0x0003)

// Averaging Mode - Configuration register bits
// Determines the number of samples that are collected and averaged.
#define INA260_CONFIG_AVG_1     (0) 
#define INA260_CONFIG_AVG_4     (0x0200)
#define INA260_CONFIG_AVG_16    (0x0400)
#define INA260_CONFIG_AVG_64    (0x0600) 
#define INA260_CONFIG_AVG_128   (0x0800)
#define INA260_CONFIG_AVG_256   (0x0A00)
#define INA260_CONFIG_AVG_512   (0x0C00)
#define INA260_CONFIG_AVG_1024  (0x0E00)

// Conversion Time bits - Configuration register bits
// Sets the conversion time for the bus voltage and current measurement.
#define INA260_CONFIG_CT_140us  (0)
#define INA260_CONFIG_CT_204us  (0x0048)
#define INA260_CONFIG_CT_332us  (0x0090)
#define INA260_CONFIG_CT_588us  (0x00D8) 
#define INA260_CONFIG_CT_1100us (0x0120)
#define INA260_CONFIG_CT_2116us (0x0168) 
#define INA260_CONFIG_CT_4156us (0x01B0)
#define INA260_CONFIG_CT_8244us (0x01F8)


//--------------------------------------------------------------------------
// external definitions

extern float32_t ina260_mWh;  // mWatt Hours defined in ina260.c
extern float32_t ina260_mAh;  // mAmp Hours defined in ina260.c

extern int ina260_init(uint16_t config, uint16_t alarm, uint8_t send_com_msg);
extern int ina260_stop(void);
extern int ina260_read(float32_t *mA, float32_t *V, uint32_t timeout);


#endif /* INA260_H_ */