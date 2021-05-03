/*
 * wispr.c
 *
 * Created: 12/24/2019
 *  Author: Chris
 */ 

#include <stdio.h>
#include <string.h>
#include "wispr.h"

#ifdef __SAM4SD32B__
#include <gpbr.h>
#endif

/*
 * Notes:
 * The SAM4s memory words are little-endian.
 * So everything written to data storage is in little-endian word format.
 * The parse and serialization functions explicitly read/write words 
 * as bytes in l-e order to avoid and cross-platform word alignment issues.
 * Any changes have to be done carefully to preserve the correct buffer indexing.  
 */

int wispr_parse_data_header(uint8_t *buf, wispr_data_header_t *hdr)
{
    if ( strncmp((const char *)buf, "WISPR2", 6) != 0 ) {
      fprintf(stdout, "wispr_parse_data_header: unrecognized header.\n");
      return(0);
	}
	
	hdr->name[0] = buf[0];
	hdr->name[1] = buf[1];
	hdr->name[2] = buf[2];
	hdr->name[3] = buf[3];
	hdr->name[4] = buf[4];
	hdr->name[5] = buf[5];

	hdr->version[0] = buf[6];
	hdr->version[1] = buf[7];

	hdr->type = (uint8_t)buf[8];

	hdr->second  = ((uint32_t)buf[9] << 0);
	hdr->second |= ((uint32_t)buf[10] << 8);
	hdr->second |= ((uint32_t)buf[11] << 16);
	hdr->second |= ((uint32_t)buf[12] << 24);

	hdr->usec  = ((uint32_t)buf[13] << 0);
	hdr->usec |= ((uint32_t)buf[14] << 8);
	hdr->usec |= ((uint32_t)buf[15] << 16);
	hdr->usec |= ((uint32_t)buf[16] << 24);
	
	hdr->settings[0] = buf[17];
	hdr->settings[1] = buf[18];
	hdr->settings[2] = buf[19];
	hdr->settings[3] = buf[20];

	hdr->buffer_size  = ((uint16_t)buf[21] << 0);
	hdr->buffer_size |= ((uint16_t)buf[22] << 8);
	
	hdr->sample_size = (uint8_t)buf[23];

	hdr->samples_per_buffer  = ((uint16_t)buf[24] << 0);
	hdr->samples_per_buffer |= ((uint16_t)buf[25] << 8);

	hdr->sampling_rate  = ((uint32_t)buf[26] << 0);
	hdr->sampling_rate |= ((uint32_t)buf[27] << 8);
	hdr->sampling_rate |= ((uint32_t)buf[28] << 16);
	hdr->sampling_rate |= ((uint32_t)buf[29] << 24);

	return(WISPR_DATA_HEADER_SIZE);
}


int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf)
{
	// update the data header info

	buf[0]  = 'W';
	buf[1]  = 'I';
	buf[2]  = 'S';
	buf[3]  = 'P';
	buf[4]  = 'R';
	buf[5]  = '2';
	
	buf[6]  = (uint8_t)(hdr->version[0]);
	buf[7]  = (uint8_t)(hdr->version[1]);
	
	buf[8]  = (uint8_t)(hdr->type);

	buf[9]  = (uint8_t)(hdr->second >> 0);
	buf[10] = (uint8_t)(hdr->second >> 8);
	buf[11] = (uint8_t)(hdr->second >> 16);
	buf[12] = (uint8_t)(hdr->second >> 24);

	buf[13] = (uint8_t)(hdr->usec >> 0);
	buf[14] = (uint8_t)(hdr->usec >> 8);
	buf[15] = (uint8_t)(hdr->usec >> 16);
	buf[16] = (uint8_t)(hdr->usec >> 24);

	buf[17] = (uint8_t)(hdr->settings[0]);
	buf[18] = (uint8_t)(hdr->settings[1]);
	buf[19] = (uint8_t)(hdr->settings[2]);
	buf[20] = (uint8_t)(hdr->settings[3]);
	
	buf[21] = (uint8_t)(hdr->buffer_size >> 0);
	buf[22] = (uint8_t)(hdr->buffer_size >> 8);
	
	buf[23] = (uint8_t)(hdr->sample_size);
	
	buf[24] = (uint8_t)(hdr->samples_per_buffer >> 0);
	buf[25] = (uint8_t)(hdr->samples_per_buffer >> 8);
	
	buf[26] = (uint8_t)(hdr->sampling_rate >> 0);
	buf[27] = (uint8_t)(hdr->sampling_rate >> 8);
	buf[28] = (uint8_t)(hdr->sampling_rate >> 16);
	buf[29] = (uint8_t)(hdr->sampling_rate >> 24);

	buf[30] = (uint8_t)(hdr->channels);

	// header checksum 
	//uint8_t chksum2 = 0;
	//for(int n=0; n<28; n++) chksum2 += buf[n];
	//buf[30]  = chksum2;
	
	// data checksum
	buf[31]  = hdr->data_chksum; 
	
	// update size of record
	return(32);
}

void wispr_print_data_header(wispr_data_header_t *hdr)
{
	fprintf(stdout, "WISPR %d.%d Data Header: ", hdr->version[1], hdr->version[0]);
	fprintf(stdout, "- time: %s\r\n", epoch_time_string(hdr->second));
	fprintf(stdout, "- type: %d\r\n", hdr->type);
	fprintf(stdout, "- settings: %d %d %d %d\r\n",hdr->settings[0],hdr->settings[1],hdr->settings[2],hdr->settings[3]);
	fprintf(stdout, "- bytes per buffer: %d\r\n", hdr->buffer_size);
	fprintf(stdout, "- bytes per sample: %d\r\n", hdr->sample_size);
	fprintf(stdout, "- samples per buffer: %d\r\n", hdr->samples_per_buffer);
	fprintf(stdout, "- sampling rate: %d\r\n", hdr->sampling_rate);
}

int wispr_parse_config(uint8_t *buf, wispr_config_t *hdr)
{
    if ( strncmp((const char *)buf, "WISPR2", 6) != 0 ) {
		fprintf(stdout, "wispr_parse_config: unrecognized config\n");
		return(0);
	}

	hdr->name[0] = buf[0];
	hdr->name[1] = buf[1];
	hdr->name[2] = buf[2];
	hdr->name[3] = buf[3];
	hdr->name[4] = buf[4];
	hdr->name[5] = buf[5];
	
	hdr->version[0] = buf[6];
	hdr->version[1] = buf[7];
	
	hdr->state = buf[8];
	hdr->mode = buf[9];

	hdr->epoch  = ((uint32_t)buf[10]); 
	hdr->epoch |= ((uint32_t)buf[11] << 8); 
	hdr->epoch |= ((uint32_t)buf[12] << 16);
	hdr->epoch |= ((uint32_t)buf[13] << 24);

	hdr->active_sd_card = (uint8_t)buf[14];

	hdr->adc.sample_size = (uint8_t)buf[15];

	hdr->adc.buffer_size  = ((uint16_t)buf[16]);
	hdr->adc.buffer_size |= ((uint16_t)buf[17] << 8);
	
	hdr->adc.samples_per_buffer  = ((uint16_t)buf[18]);
	hdr->adc.samples_per_buffer |= ((uint16_t)buf[19] << 8);

	hdr->adc.sampling_rate  = ((uint32_t)buf[20]);
	hdr->adc.sampling_rate |= ((uint32_t)buf[21] << 8);
	hdr->adc.sampling_rate |= ((uint32_t)buf[22] << 16);
	hdr->adc.sampling_rate |= ((uint32_t)buf[23] << 24);

	// 8 bytes of settings
	hdr->adc.gain = buf[24];
	hdr->adc.decimation = buf[25];
	// 6 more unused

	hdr->acquisition_time  = ((uint16_t)buf[32]);
	hdr->acquisition_time |= ((uint16_t)buf[33] << 8);

	hdr->sleep_time  = ((uint16_t)buf[34]);
	hdr->sleep_time |= ((uint16_t)buf[35] << 8);

	hdr->psd.size  = ((uint16_t)buf[36]);
	hdr->psd.size |= ((uint16_t)buf[37] << 8);

	hdr->psd.overlap  = ((uint16_t)buf[38]);
	hdr->psd.overlap |= ((uint16_t)buf[39] << 8);

	return(40);
}


int wispr_serialize_config(wispr_config_t *hdr, uint8_t *buf)
{
	buf[0]  = 'W';
	buf[1]  = 'I';
	buf[2]  = 'S';
	buf[3]  = 'P';
	buf[4]  = 'R';
	buf[5]  = '2';
		
	buf[6]  = (uint8_t)(hdr->version[0]);
	buf[7]  = (uint8_t)(hdr->version[1]);
	
	buf[8]  = (uint8_t)(hdr->state);
	buf[9]  = (uint8_t)(hdr->mode);

	buf[10] = (uint8_t)(hdr->epoch >> 0);
	buf[11] = (uint8_t)(hdr->epoch >> 8);
	buf[12] = (uint8_t)(hdr->epoch >> 16);
	buf[13] = (uint8_t)(hdr->epoch >> 24);
	
	buf[14] = (uint8_t)(hdr->active_sd_card);
	buf[15] = (uint8_t)(hdr->adc.sample_size);
	
	buf[16] = (uint8_t)(hdr->adc.buffer_size >> 0);
	buf[17] = (uint8_t)(hdr->adc.buffer_size >> 8);
	
	buf[18] = (uint8_t)(hdr->adc.samples_per_buffer >> 0);
	buf[19] = (uint8_t)(hdr->adc.samples_per_buffer >> 8);
	
	buf[20] = (uint8_t)(hdr->adc.sampling_rate >> 0);
	buf[21] = (uint8_t)(hdr->adc.sampling_rate >> 8);
	buf[22] = (uint8_t)(hdr->adc.sampling_rate >> 16);
	buf[23] = (uint8_t)(hdr->adc.sampling_rate >> 24);

	buf[24] = (uint8_t)(hdr->adc.gain);
	buf[25] = (uint8_t)(hdr->adc.decimation);
	buf[26] = (uint8_t)0;  // reserved
	buf[27] = (uint8_t)0;  // reserved
	buf[28] = (uint8_t)0;  // reserved
	buf[29] = (uint8_t)0;  // reserved
	buf[30] = (uint8_t)0;  // reserved
	buf[31] = (uint8_t)0;  // reserved

	buf[32] = (uint8_t)(hdr->acquisition_time >> 0);
	buf[33] = (uint8_t)(hdr->acquisition_time >> 8);
	
	buf[34] = (uint8_t)(hdr->sleep_time >> 0);
	buf[35] = (uint8_t)(hdr->sleep_time >> 8);
	
	buf[36] = (uint8_t)(hdr->psd.size >> 0);
	buf[37] = (uint8_t)(hdr->psd.size >> 8);
	
	buf[38] = (uint8_t)(hdr->psd.overlap >> 0);
	buf[39] = (uint8_t)(hdr->psd.overlap >> 8);

	// update the data record size
	return(40);
}

void wispr_update_data_header(wispr_config_t *wispr, wispr_data_header_t *hdr)
{	
	// Initialize local data header structure with the config
	// this is used to update the current data buffer header
	hdr->version[0] = wispr->version[0];
	hdr->version[1] = wispr->version[1];
	hdr->settings[ADC_SETTINGS_INDEX_GAIN] = wispr->adc.gain;
	hdr->settings[ADC_SETTINGS_INDEX_DF] = wispr->adc.decimation;
	hdr->settings[ADC_SETTINGS_INDEX_2] = 0;
	hdr->settings[ADC_SETTINGS_INDEX_3] = 0;
	hdr->sample_size = wispr->adc.sample_size; // number of bytes per sample
	hdr->buffer_size = wispr->adc.buffer_size; // number of bytes in an adc record buffer
	hdr->samples_per_buffer = wispr->adc.samples_per_buffer;  // number of samples in a buffer
	hdr->sampling_rate = wispr->adc.sampling_rate; // samples per second
	hdr->second = wispr->epoch; // epoch time stamp
	hdr->usec = 0;
	hdr->header_chksum = 0;
	hdr->data_chksum = 0;
}

