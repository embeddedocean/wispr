/*
 *  pmel.h
 */
#ifndef PMEL_H
#define PMEL_H

#include <status_codes.h>
#include "wispr.h"
#include "com.h"
#include "gps.h"
#include "spectrum.h"

// max values
#define PMEL_MAX_SLEEP (60*60*24)
#define PMEL_MAX_PAUSE (60*60)
#define PMEL_MAX_PSD_DARATION (60*60)
#define PMEL_MAX_SAMPLING_RATE (200000)

// default values
#define PMEL_SAMPLING_RATE 50000
#define PMEL_ADC_DECIMATION 8
#define PMEL_GAIN 0
#define PMEL_FFT_SIZE 1024
#define PMEL_PSD_DURATION 60

// Time to wait for ACK after sending message
#define PMEL_ACK_TIMEOUT_MSEC 10000

// Number of time to retry sending a message that requires an ack
#define PMEL_NUMBER_RETRIES 4

/* pmel command message status codes */
#define PMEL_NONE 0
#define PMEL_UNKNOWN 0
#define PMEL_ERROR 1

/* pmel message types  (8 bits) */
#define PMEL_UNKNOWN 0
#define PMEL_ACK 1
#define PMEL_NAK 99 
#define PMEL_EXI 2
#define PMEL_RUN 3
#define PMEL_PAU 4
#define PMEL_RST 5
#define PMEL_SLP 6
#define PMEL_STA 7
#define PMEL_GPS 8
#define PMEL_TME 9
#define PMEL_WTM 10
#define PMEL_NGN 11
#define PMEL_SDF 12
#define PMEL_PSD 13
#define PMEL_PWR 14
#define PMEL_TOU 15
#define PMEL_ADC 16

//
// Application specific control structure
//
typedef struct {
	char instrument_id[8];
	char location_id[8];
	uint8_t version[2];  // software version
	uint32_t second; // linux time in seconds
	uint32_t file_size; // number of block (512 byte) per file
	gps_t gps; // latest gps update
	float32_t volts; // battery voltage
	float32_t amps; //  
	float32_t free; // percentage of free space on active sd card
} pmel_control_t;


// prototypes
extern int pmel_init(wispr_config_t *config);
extern int pmel_control (wispr_config_t *config, uint16_t timeout);
extern int pmel_msg_type (char *buf);

extern int pmel_wait_for_ack (wispr_config_t *config);
extern int pmel_send_wait_for_ack(wispr_config_t *config, char *msg);

extern int pmel_request_gps(wispr_config_t *config, uint16_t timeout);
extern int pmel_request_gain(wispr_config_t *config, uint16_t timeout);
extern int pmel_request_spectrum(wispr_config_t *config, char *msg);

extern int pmel_set_sleep(wispr_config_t *config, char *buf);
extern int pmel_set_pause(wispr_config_t *config, char *buf);
extern int pmel_set_gain(wispr_config_t *config, char *buf);
extern int pmel_set_time(wispr_config_t *config, char *buf);
extern int pmel_set_timeout(wispr_config_t *config, char *buf);
extern int pmel_set_adc(wispr_config_t *config, char *buf);

extern int pmel_send_spectrum(wispr_config_t *config, float32_t *psd_average, uint16_t nbins, uint8_t *buffer, pmel_control_t *pmel);
extern int pmel_send_sdb(wispr_config_t *config, float32_t *psd_average, uint16_t nbins);
extern int pmel_send_sd_usage(wispr_config_t *config);
extern int pmel_send_status(wispr_config_t *config);
extern int pmel_send_power_usage(wispr_config_t *config);
extern int pmel_send_time(wispr_config_t *config);

extern char *pmel_time_string(uint32_t epoch);
extern void pmel_filename(char *name, char *prefix, char *suffix, rtc_time_t *dt);
extern int pmel_file_header(char *buf, wispr_config_t *cfg, wispr_data_header_t *hdr, pmel_control_t *pmel);
extern int pmel_update_config (wispr_config_t *pmel, wispr_config_t *wispr);


#endif /* _PMEL_H */

