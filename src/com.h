/*
 *  com.h
 */
#ifndef _COM_H
#define _COM_H

#include <status_codes.h>

#define COM_MAX_MESSAGE_SIZE 64
#define COM_MESSAGE_PREFIX '$'
#define COM_MESSAGE_SUFFIX '*'

/* message types */
#define COM_UNKNOWN 0
#define COM_EXIT 1
#define COM_RUN 2
#define COM_PAUSE 3
#define COM_RESET 4
#define COM_SLEEP 5
#define COM_GPS 6
#define COM_TIME 7
#define COM_GAIN 8
#define COM_STATUS 9
#define COM_SDF 10

// this contains all information sent in all the message types
typedef struct {
	uint32_t sec; // msg time
	int type;
	int state;
	int gain;
	float lat;
	float lon;
	float depth;
} wispr_com_msg_t;

// prototypes
extern int com_init(int port, uint32_t baud);
extern void com_stop(int port);
//extern int com_read_msg(int port, char *buf, int timeout);
extern int com_read_msg(int port, char *buf);
extern int com_write_msg(int port, char *buf);
extern int com_parse_msg(wispr_com_msg_t *msg, char *buf, int len);

//extern int com_send_dtx(wispr_com_t *com, int max_ndtx);
//extern void com_reset_msg(wispr_com_t *com);


#endif /* _COM_H */

