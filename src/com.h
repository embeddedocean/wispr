/*
 *  com.h
 */
#ifndef _COM_H
#define _COM_H

#include <status_codes.h>

#define COM_MAX_MESSAGE_SIZE 64
#define COM_MESSAGE_PREFIX '$'
#define COM_MESSAGE_SUFFIX '*'

/* status/error types */
#define COM_NO_MSG 0
#define COM_VALID_MSG 1
#define COM_INVALID_CRC -1
#define COM_TIMEOUT -2

/* message types */
#define COM_UNKNOWN 0
#define COM_EXIT 1
#define COM_RUN 2
#define COM_PAUSE 3
#define COM_RESET 4
#define COM_SLEEP 5
#define COM_STATUS 6
#define COM_SET 7
#define COM_GPS 8
#define COM_TIME 9
#define COM_GAIN 10
#define COM_SDF 11

// this contains the application specific message information
//typedef struct {
//	uint32_t sec; // msg time
//	int type;
//	int state;
//	int gain;
//	float lat;
//	float lon;
//	float depth;
//} wispr_com_msg_t;

// prototypes
extern int com_init(int port, uint32_t baud);
extern void com_stop(int port);
extern int com_read_msg(int port, char *buf, int timeout);
extern int com_write_msg(int port, const char *buf);

//extern int com_parse_msg(wispr_com_msg_t *msg, char *buf, int len);
//extern int com_request_gps(wispr_com_msg_t *msg, uint16_t timeout);
//extern int com_request_gain(wispr_com_msg_t *msg, uint16_t timeout);

//extern int com_send_dtx(wispr_com_t *com, int max_ndtx);
//extern void com_reset_msg(wispr_com_t *com);


#endif /* _COM_H */

