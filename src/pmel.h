/*
 *  pmel.h
 */
#ifndef PMEL_H
#define PMEL_H

#include <status_codes.h>

// prototypes
extern int pmel_parse_msg(wispr_com_msg_t *msg, char *buf, int len);
extern int pmel_request_gps(wispr_com_msg_t *msg, uint16_t timeout);
extern int pmel_request_gain(wispr_com_msg_t *msg, uint16_t timeout);
extern int pmel_file_header(char *buf, wispr_config_t *cfg, wispr_data_header_t *hdr);


#endif /* _PMEL_H */

