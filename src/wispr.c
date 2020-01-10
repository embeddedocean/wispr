/*
 * wispr.c
 *
 * Created: 12/24/2019 5:02:40 AM
 *  Author: Chris
 */ 

#include <stdio.h>
#include "wispr.h"

int wispr_parse_data_header(uint8_t *buf, wispr_data_header_t *hdr)
{
    if ((buf[0] != 'W') && (buf[1] != 'I') && (buf[2] != 'S') && (buf[3] != 'P') && (buf[4] != 'R')) {
      fprintf(stdout, "wispr_parse_header: unknown header %c%c%c%c%c\n", buf[0],buf[1],buf[2],buf[3],buf[4]);
      return(0);
    }

	hdr->name[0] = buf[0];
	hdr->name[1] = buf[1];
	hdr->name[2] = buf[2];
	hdr->name[3] = buf[3];
	hdr->name[4] = buf[4];
	hdr->version[0] = buf[5];
	hdr->version[1] = buf[6];
	hdr->size = buf[7];
	hdr->settings[0] = buf[8];
	hdr->settings[1] = buf[9];
	hdr->bytes_per_sample = buf[10];
	hdr->num_samples = buf[11] | (buf[12] << 8);
	hdr->sampling_rate = (uint32_t)buf[13] | ((uint32_t)buf[14] << 8) | ((uint32_t)buf[15] << 16) | ((uint32_t)buf[16] << 24);
	hdr->year = buf[17];
	hdr->month = buf[18];
	hdr->day = buf[19];
	hdr->hour = buf[20];
	hdr->minute = buf[21];
	hdr->second = buf[22];
	hdr->usec = buf[23] | (buf[24] << 8) | (buf[25] << 16) | (buf[26] << 24);

	return(ADC_HEADER_SIZE);
}


int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf)
{
	// update the data header info
	uint8_t chksum2 = 0;

	buf[0]   = 'W';
	buf[1]   = 'I';
	buf[2]   = 'S';
	buf[3]   = 'P';
	buf[4]   = 'R';
	buf[5]   = (uint8_t)(hdr->version[0]);
	buf[6]   = (uint8_t)(hdr->version[1]);
	buf[7]   = (uint8_t)(hdr->size);
	buf[8]   = (uint8_t)(hdr->settings[0]);
	buf[9]   = (uint8_t)(hdr->settings[1]);
	buf[10]  = (uint8_t)(hdr->bytes_per_sample);
	buf[11]  = (uint8_t)(hdr->num_samples >> 0);
	buf[12]  = (uint8_t)(hdr->num_samples >> 8);
	buf[13]  = (uint8_t)(hdr->sampling_rate >> 0);
	buf[14]  = (uint8_t)(hdr->sampling_rate >> 8);
	buf[15]  = (uint8_t)(hdr->sampling_rate >> 16);
	buf[16]  = (uint8_t)(hdr->sampling_rate >> 24);
	buf[17]  = (uint8_t)(hdr->year);
	buf[18]  = (uint8_t)(hdr->month);
	buf[19]  = (uint8_t)(hdr->day);
	buf[20]  = (uint8_t)(hdr->hour);
	buf[21]  = (uint8_t)(hdr->minute);
	buf[22]  = (uint8_t)(hdr->second);
	buf[23]  = (uint8_t)(hdr->usec >> 0);
	buf[24]  = (uint8_t)(hdr->usec >> 8);
	buf[25]  = (uint8_t)(hdr->usec >> 16);
	buf[26]  = (uint8_t)(hdr->usec >> 24);
	// header checksum 
	for(int n=0; n<27; n++) chksum2 += buf[n];
	buf[27]  = chksum2;
	// data checksum
	buf[28]  = hdr->data_chksum; 
	buf[29]  = 0xaa;  // padding
	
	// update size of record
	hdr->size = 30;
	buf[7]   = (uint8_t)(hdr->size);
	return(30);
}

void wispr_print_data_header(wispr_data_header_t *hdr)
{
	fprintf(stdout, "WISPR %d.%d: ", hdr->version[0], hdr->version[1]);
	fprintf(stdout, "%02d/%02d/%02d %02d-%02d-%02d\r\n",
		hdr->year,hdr->month,hdr->day,hdr->hour,hdr->minute,hdr->second);
	fprintf(stdout, "- header size: %d\r\n", hdr->size);
	fprintf(stdout, "- settings: %02x %02x \r\n", hdr->settings[0], hdr->settings[1]);
	fprintf(stdout, "- bytes per sample: %d\r\n", hdr->bytes_per_sample);
	fprintf(stdout, "- samples per buffer: %d\r\n", hdr->num_samples);
	fprintf(stdout, "- sampling rate: %d\r\n", hdr->sampling_rate);
}

//
// 
//
int wispr_parse_config(uint8_t *buf, wispr_config_t *hdr)
{
	if ((buf[0] != 'W') && (buf[1] != 'I') && (buf[2] != 'S') && (buf[3] != 'P') && (buf[4] != 'R')) {
		fprintf(stdout, "wispr_parse_config: unrecognized name\n");
		return(0);
	}

	hdr->name[0] = buf[0];
	hdr->name[1] = buf[1];
	hdr->name[2] = buf[2];
	hdr->name[3] = buf[3];
	hdr->name[4] = buf[4];
	hdr->version[0] = buf[5];
	hdr->version[1] = buf[6];
	hdr->size = buf[7];
	hdr->year = buf[8];
	hdr->month = buf[9];
	hdr->day = buf[10];
	hdr->hour = buf[11];
	hdr->minute = buf[12];
	hdr->second = buf[13];
	hdr->bytes_per_sample = buf[14];
	hdr->blocks_per_record = buf[15];
	hdr->samples_per_record = buf[16] | (buf[17] << 8);
	hdr->sampling_rate = (uint32_t)buf[18] | ((uint32_t)buf[19] << 8) | ((uint32_t)buf[20] << 16) | ((uint32_t)buf[21] << 24);
	hdr->adc_settings[0] = buf[22];
	hdr->adc_settings[1] = buf[23];
	hdr->preamp_settings[0] = buf[24];
	hdr->preamp_settings[1] = buf[25];
	return(26);
}


int wispr_serialize_config(wispr_config_t *hdr, uint8_t *buf)
{
	buf[0]   = 'W';
	buf[1]   = 'I';
	buf[2]   = 'S';
	buf[3]   = 'P';
	buf[4]   = 'R';
	buf[5]   = (uint8_t)(hdr->version[0]);
	buf[6]   = (uint8_t)(hdr->version[1]);
	buf[7]   = (uint8_t)(hdr->size);
	buf[8]  = (uint8_t)(hdr->year);
	buf[9]  = (uint8_t)(hdr->month);
	buf[10]  = (uint8_t)(hdr->day);
	buf[11]  = (uint8_t)(hdr->hour);
	buf[12]  = (uint8_t)(hdr->minute);
	buf[13]  = (uint8_t)(hdr->second);
	buf[14]  = (uint8_t)(hdr->bytes_per_sample);
	buf[15]  = (uint8_t)(hdr->blocks_per_record);
	buf[16]  = (uint8_t)(hdr->samples_per_record >> 0);
	buf[17]  = (uint8_t)(hdr->samples_per_record >> 8);
	buf[18]  = (uint8_t)(hdr->sampling_rate >> 0);
	buf[19]  = (uint8_t)(hdr->sampling_rate >> 8);
	buf[20]  = (uint8_t)(hdr->sampling_rate >> 16);
	buf[21]  = (uint8_t)(hdr->sampling_rate >> 24);
	buf[22]   = (uint8_t)(hdr->adc_settings[0]);
	buf[23]   = (uint8_t)(hdr->adc_settings[1]);
	buf[24]   = (uint8_t)(hdr->preamp_settings[0]);
	buf[25]   = (uint8_t)(hdr->preamp_settings[1]);

	// update the data record size
	hdr->size = 26;
	buf[7]   = (uint8_t)(hdr->size);
	return(hdr->size);
}

void wispr_print_config(wispr_config_t *hdr)
{
	fprintf(stdout, "\r\nWISPR %d.%d: configuration time ", hdr->version[0], hdr->version[1]);
	fprintf(stdout, "%02d/%02d/%02d %02d-%02d-%02d\r\n",
	   hdr->year,hdr->month,hdr->day,hdr->hour,hdr->minute,hdr->second);
	fprintf(stdout, "- header size: %d\r\n", hdr->size);
	fprintf(stdout, "- adc settings: %02x %02x \r\n", hdr->adc_settings[0], hdr->adc_settings[1]);
	fprintf(stdout, "- preamp settings: %02x %02x \r\n", hdr->preamp_settings[0], hdr->preamp_settings[1]);
	fprintf(stdout, "- bytes per sample: %d\r\n", hdr->bytes_per_sample);
	fprintf(stdout, "- samples per buffer: %d\r\n", hdr->samples_per_record);
	fprintf(stdout, "- sampling rate: %d\r\n", hdr->sampling_rate);
	
	float buffer_duration =  (float)hdr->samples_per_record / (float)hdr->sampling_rate;
	fprintf(stdout, "- buffer duration: %lu msec\n\r", (uint32_t)(1000.0*buffer_duration));
	fprintf(stdout, "- number blocks per data buffer: %lu \n\r", hdr->blocks_per_record);

}

