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

/* pmel command message status codes */
#define PMEL_NONE 0
#define PMEL_UNKNOWN 0
#define PMEL_ERROR 1

/* pmel message types  (8 bits) */
#define PMEL_UNKNOWN 0
#define PMEL_ACK 1
#define PMEL_NACK 99 
#define PMEL_EXIT 2
#define PMEL_RUN 3
#define PMEL_PAUSE 4
#define PMEL_RESET 5
#define PMEL_SLEEP 6
#define PMEL_STATUS 7
#define PMEL_GPS 8
#define PMEL_TIME 9
#define PMEL_GAIN 10
#define PMEL_SDF 11
#define PMEL_PSD 12

//
// Application specific control structure
//
typedef struct {
	char instrument_id[8];
	char location_id[8];
	uint8_t version[2];  // software version
	uint32_t second; // linux time in seconds
	uint32_t file_size; // number of block (512 byte) per file
	//spectrum_t psd; // latest psd settings
	gps_t gps; // latest gps update
	float32_t volts; // battery voltage
	float32_t amps; //  
	float32_t free; // percentage of free space on active sd card
} pmel_control_t;


// prototypes
extern int pmel_init(wispr_config_t *config);
extern int pmel_control (wispr_config_t *config, uint16_t timeout);
extern int pmel_request_gps(wispr_config_t *config, uint16_t timeout);
extern int pmel_request_gain(wispr_config_t *config, uint16_t timeout);
extern int pmel_msg_type (char *buf);
extern int pmel_transmit_spectrum(wispr_config_t *config, float32_t *psd_average, uint16_t nbins, uint8_t *buffer, pmel_control_t *pmel);
extern int pmel_send_sdb(wispr_config_t *config, float32_t *psd_average, uint16_t nbins);
extern int pmel_wait_for_ack (wispr_config_t *config, uint16_t timeout_sec);

extern char *pmel_time_string(uint32_t epoch);
extern void pmel_filename(char *name, char *prefix, char *suffix, rtc_time_t *dt);
extern int pmel_file_header(char *buf, wispr_config_t *cfg, wispr_data_header_t *hdr, pmel_control_t *pmel);
extern int pmel_update_config (wispr_config_t *pmel, wispr_config_t *wispr);


#endif /* _PMEL_H */

