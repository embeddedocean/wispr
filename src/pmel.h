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

/* pmel command message types */
#define PMEL_UNKNOWN 0
#define PMEL_EXIT 1
#define PMEL_RUN 2
#define PMEL_PAUSE 3
#define PMEL_RESET 4
#define PMEL_SLEEP 5
#define PMEL_STATUS 6
#define PMEL_SET 7
#define PMEL_GPS 8
#define PMEL_TIME 9
#define PMEL_GAIN 10
#define PMEL_SDF 11
#define PMEL_PSD 12

//
// Application specific control structure
//
typedef struct {
	uint8_t  state;
	uint8_t  mode;
	uint32_t second; // linux time in seconds
	uint16_t acquisition_time; // time in seconds of the adc sampling window
	uint16_t sleep_time; // time in seconds between adc records (must be >= window)
	uint32_t file_size; // number of block (512 byte) per file
	uint8_t gain; // adc gain setting
	spectrum_t psd;
	gps_t gps; // latest gps update
} pmel_control_t;


// prototypes
extern int pmel_init(wispr_config_t *config);
extern int pmel_control (wispr_config_t *config, uint16_t timeout);
extern int pmel_request_gps(wispr_config_t *config, uint16_t timeout);
extern int pmel_request_gain(wispr_config_t *config, uint16_t timeout);
extern int pmel_msg_type (char *buf);
extern int pmel_transmit_spectrum(wispr_config_t *config, float32_t *psd_average, uint16_t nbins);
extern int pmel_send_sdb(wispr_config_t *config, float32_t *psd_average, uint16_t nbins);

extern void pmel_filename(char *name, char *prefix, char *suffix, rtc_time_t *dt);
extern int pmel_file_header(char *buf, wispr_config_t *cfg, wispr_data_header_t *hdr);
extern int pmel_update_config (wispr_config_t *pmel, wispr_config_t *wispr);


#endif /* _PMEL_H */

