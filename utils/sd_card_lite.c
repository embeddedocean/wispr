/**
 * sd_card_lite.c: sd card utility functions
 * 
 Notes:
 *
 */

#include <string.h>
#include <stdio.h>

#include "sd_card_lite.h"
#include "rtc_time.h"

void sd_card_print_header(sd_card_t *hdr)
{
    fprintf(stdout, "\r\nWISPR %d.%d Card Header\r\n", hdr->version[0], hdr->version[1]);
    fprintf(stdout, "- addr of start block:          %d\r\n", hdr->start);
    fprintf(stdout, "- addr of end block:            %d\r\n", hdr->end);
    fprintf(stdout, "- addr of current write block:  %d\r\n", hdr->write_addr);
    fprintf(stdout, "- addr of current read block:   %d\r\n", hdr->read_addr);
    fprintf(stdout, "- time last header was written: %s\r\n", epoch_time_string(hdr->epoch));
}

int sd_card_parse_header(uint8_t *buf, sd_card_t *hdr)
{
	if ( strncmp((const char *)buf, "WISPR", 5) != 0 ) {
		fprintf(stdout, "wispr_sd_card_parse_header: unrecognized header\r\n");
		return(0);
	}

	hdr->version[0] = buf[6];
	hdr->version[1] = buf[7];

	hdr->label[0] = buf[8];
	hdr->label[1] = buf[9];
	hdr->label[2] = buf[10];
	hdr->label[3] = buf[11];
	hdr->label[4] = buf[12];
	hdr->label[5] = buf[13];
	hdr->label[6] = buf[14];
	hdr->label[7] = buf[15];

	hdr->start  =  (uint32_t)buf[16];    // addr of first block (uint32_t)
	hdr->start |= ((uint32_t)buf[17] << 8);
	hdr->start |= ((uint32_t)buf[18] << 16);
	hdr->start |= ((uint32_t)buf[19] << 24);    // addr of first block (uint32_t)

	hdr->end  =  (uint32_t)buf[20];
	hdr->end |= ((uint32_t)buf[21] << 8);
	hdr->end |= ((uint32_t)buf[22] << 16);
	hdr->end |= ((uint32_t)buf[23] << 24);    // addr of last block (uint32_t)

	hdr->write_addr   = (uint32_t)buf[24];
	hdr->write_addr |= ((uint32_t)buf[25] << 8);
	hdr->write_addr |= ((uint32_t)buf[26] << 16);
	hdr->write_addr |= ((uint32_t)buf[27] << 24);    // addr of current write block (uint32_t)
	
	hdr->read_addr  =  (uint32_t)buf[28];
	hdr->read_addr |= ((uint32_t)buf[29] << 8);
	hdr->read_addr |= ((uint32_t)buf[30] << 16);
	hdr->read_addr |= ((uint32_t)buf[31] << 24);    // addr of current write block (uint32_t)
	
	hdr->epoch  =  (uint32_t)buf[32];
	hdr->epoch |= ((uint32_t)buf[33] << 8);
	hdr->epoch |= ((uint32_t)buf[34] << 16);
	hdr->epoch |= ((uint32_t)buf[35] << 24);    // addr of current write block (uint32_t)
	return(36);
}

int sd_card_serialize_header(sd_card_t *hdr, uint8_t *buf)
{
	buf[0]  = 'W';
	buf[1]  = 'I';
	buf[2]  = 'S';
	buf[3]  = 'P';
	buf[4]  = 'R';
	buf[5]  = '2';

	buf[6]  = (uint8_t)(hdr->version[0]);
	buf[7]  = (uint8_t)(hdr->version[1]);

	buf[8]  = hdr->label[0];
	buf[9]  = hdr->label[1];
	buf[10] = hdr->label[2];
	buf[11] = hdr->label[3];
	buf[12] = hdr->label[4];
	buf[13] = hdr->label[5];
	buf[14] = hdr->label[6];
	buf[15] = hdr->label[7];

	buf[16] = (uint8_t)(hdr->start >> 0);
	buf[17] = (uint8_t)(hdr->start >> 8);
	buf[18] = (uint8_t)(hdr->start >> 16);
	buf[19] = (uint8_t)(hdr->start >> 24);
	
	buf[20] = (uint8_t)(hdr->end >> 0);
	buf[21] = (uint8_t)(hdr->end >> 8);
	buf[22] = (uint8_t)(hdr->end >> 16);
	buf[23] = (uint8_t)(hdr->end >> 24);

	buf[24] = (uint8_t)(hdr->write_addr >> 0);
	buf[25] = (uint8_t)(hdr->write_addr >> 8);
	buf[26] = (uint8_t)(hdr->write_addr >> 16);
	buf[27] = (uint8_t)(hdr->write_addr >> 24);
	
	buf[28] = (uint8_t)(hdr->read_addr >> 0);
	buf[29] = (uint8_t)(hdr->read_addr >> 8);
	buf[30] = (uint8_t)(hdr->read_addr >> 16);
	buf[31] = (uint8_t)(hdr->read_addr >> 24);

	buf[32] = (uint8_t)(hdr->epoch >> 0);
	buf[33] = (uint8_t)(hdr->epoch >> 8);
	buf[34] = (uint8_t)(hdr->epoch >> 16);
	buf[35] = (uint8_t)(hdr->epoch >> 24);

	return(36);
}

