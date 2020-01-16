/*
 * eopch.h
 *
 * Created: 1/16/2020 8:41:28 AM
 *  Author: Chris
 */ 

#ifndef EOPCH_H_
#define EOPCH_H_

#include "rtc_time.h"

extern uint32_t rtc_time_to_epoch(rtc_time_t *t);
extern uint32_t time_to_epoch(uint8_t yr, uint8_t mn, uint8_t dy, uint8_t hr, uint8_t mi, uint8_t se);
extern void epoch_to_time(uint32_t epoch, uint8_t *yr, uint8_t *mn, uint8_t *dy, uint8_t *hr, uint8_t *mi, uint8_t *se);
extern void epoch_to_rtc_time(rtc_time_t *t, uint32_t epoch);
extern void epoch_to_time_string(uint32_t epoch, char *str);
extern char *epoch_time_string(uint32_t epoch);


#endif /* EOPCH_H_ */