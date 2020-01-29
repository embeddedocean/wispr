/*
 * read_sdcard.c
 *
 */
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>


#include "wispr.h"
#include "wav_file.h"																																																							

uint8_t buffer[ADC_MAX_BUFFER_SIZE];
uint8_t *buffer_header = buffer;  // header is at start of buffer
uint8_t *buffer_data = &buffer[WISPR_DATA_HEADER_SIZE]; // data follows header

wispr_data_header_t hdr;
wispr_config_t cfg;
wispr_sd_card_t sd;

uint32_t find_header(int fd);

int main(int argc, char **argv)
{
//  char *input_path = "/dev/sdb";
  char *input_path = "\\\\.\\D:";
  char *output_path = ".";
  char *prefix = "WISPR";
  char out_filename[128];
  
  int c = 0;
  int buffer_size = 0;
  size_t nrd = 0;
  size_t nwrt = 0;
  size_t start_block = WISPR_SD_CARD_START_BLOCK;
  uint32_t pos = 0;

  int buffers_per_file = 100;

  int dat_count = 0;
  int psd_count = 0;

  FILE *dat_fp = NULL;
  FILE *psd_fp = NULL;

  int wav_file = 0;
  wav_file_t *psd_wf = NULL;
  wav_file_t *dat_wf = NULL;
 
  // Parse command line args
  while ((c = getopt(argc, argv, "w:b:s:i:o:p:h")) != -1) 
  {
    switch (c) 
    {
      case 's':
      {
        start_block = atoi(optarg);
        break;
      }
      case 'b':
      {
        buffers_per_file = atoi(optarg);
        break;
      }
      case 'i':
      {
        input_path = optarg;
        break;
      }
      case 'o':
      {
        output_path = optarg;
        break;
      }
      case 'p':
      {
        prefix = optarg;
        break;
      }
      case 'w':
      {
        wav_file = 1;
        break;
      }
      case 'h':
      {
        fprintf(stdout, "Read SD Card and store data to file\n");
        return 1;
      }
    }
  }
  
  // Open the device with raw samples
  //input_fd = open(input_path, (O_RDONLY | O_LARGEFILE));
  //input_fd = open(input_path, (O_RDONLY ));
  //FILE *fp = fdopen(input_fd, "r");
  FILE *fp = fopen(input_path, "r");

  if(fp == NULL) {
    fprintf(stdout, "failed to open SD Card\n");
    return -1;
  }
  fprintf(stdout, "Openned SD Card\n");

  // read the card header block
  pos = (WISPR_SD_CARD_BLOCK_SIZE * WISPR_SD_CARD_HEADER_BLOCK);
  //lseek(input_fd, pos, SEEK_SET);
  //nrd = read(input_fd, buffer, WISPR_SD_CARD_BLOCK_SIZE);
  fseek(fp, pos, SEEK_SET);
  nrd = fread(buffer, 1, WISPR_SD_CARD_BLOCK_SIZE, fp);
  if( wispr_sd_card_parse_header(buffer, &sd) == 0 ) {
     fprintf(stdout, "failed to parse card header\n");
  }

  // read the configuration block of the card
  pos = (WISPR_SD_CARD_BLOCK_SIZE * WISPR_SD_CARD_CONFIG_BLOCK);
//  lseek(input_fd, pos, SEEK_SET);
//  nrd = read(input_fd, buffer, WISPR_SD_CARD_BLOCK_SIZE);
  fseek(fp, pos, SEEK_SET);
  nrd = fread(buffer, 1, WISPR_SD_CARD_BLOCK_SIZE, fp);
  if( wispr_parse_config(buffer, &cfg) == 0 ) {
     fprintf(stdout, "failed to parse configuration\n");
  }

  wispr_print_config(&cfg);
  
  //pos = find_header(input_fd);
  start_block = WISPR_SD_CARD_START_BLOCK;
  pos = (WISPR_SD_CARD_BLOCK_SIZE * start_block);

  // seek to start of data	  
  fseek(fp, pos, SEEK_SET);
  fprintf(stdout, "Seek to file position %d (block %d)\n", pos, start_block);

  dat_count = 0;
  int go = 1;
  while(go) 
  {

    // Read data buffer header
    //nrd = read(input_fd, buffer_header, WISPR_DATA_HEADER_SIZE);
    nrd = fread(buffer_header, 1, WISPR_DATA_HEADER_SIZE, fp);
    if (nrd =! WISPR_DATA_HEADER_SIZE) {
       fprintf(stdout, "failed to read data header: %d\n", nrd);
       return -1;
    }

    // Parse data header
    if( wispr_parse_data_header(buffer_header, &hdr) == 0 ) {
       fprintf(stdout, "failed to parse data header\n");
       return -1;
       //continue;
    }

    //wispr_print_data_header(&hdr);

	// Read data buffer
    buffer_size = (size_t)hdr.block_size - WISPR_DATA_HEADER_SIZE;

    //printf("header found at %d, buffer size %d\n", ftell(fp), buffer_size);

    //nrd = read(input_fd, buffer_data, buffer_size);
    nrd = fread(buffer_data, 1, buffer_size, fp);
    if (nrd =! buffer_size) {
       fprintf(stdout, "failed to read data block: %d\n", nrd);
       return -1;
    }
	
	// open new waveform output file
    if( (dat_count == 0) && (hdr.type == WISPR_WAVEFORM) ) {
		wispr_print_data_header(&hdr);
		rtc_time_t rtc;
		epoch_to_rtc_time(&rtc, hdr.second);
		if( wav_file == 1 ) {
	         sprintf(out_filename,"%s/%s_DAT_%02d%02d%02d_%02d%02d%02d.wav", output_path, prefix, 
                rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second);
	         if(dat_wf != NULL) wav_close(dat_wf);
			 dat_wf = wav_open(out_filename, "w+", hdr.sample_size*8, hdr.sampling_rate, 1, WAVE_FORMAT_PCM);
             fprintf(stdout, "Open wav file %s\n", out_filename);
		} else {
			sprintf(out_filename,"%s/%s_%02d%02d%02d_%02d%02d%02d.dat", output_path, prefix, 
				rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second);
			// Open file to write raw data
			if(dat_fp) fclose(dat_fp);
			//dat_fd = open(out_filename, (O_WRONLY | O_CREAT));
			dat_fp = fopen(out_filename, "w");
			fprintf(stdout, "Open dat file %s\n", out_filename);
		}
	}

    if( (dat_count == 0) && (hdr.type == WISPR_SPECTRUM) ) {
		wispr_print_data_header(&hdr);
		rtc_time_t rtc;
		epoch_to_rtc_time(&rtc, hdr.second);
		if( wav_file == 1 ) {
			sprintf(out_filename,"%s/%s_PSD_%02d%02d%02d_%02d%02d%02d.wav", output_path, prefix, 
                rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second);
	        if(psd_wf != NULL) wav_close(psd_wf);
			psd_wf = wav_open(out_filename, "w+", hdr.sample_size*8, hdr.sampling_rate, 1, WAVE_FORMAT_IEEE_FLOAT);
            fprintf(stdout, "Open wav file %s\n", out_filename);
		} else {
			sprintf(out_filename,"%s/%s_%02d%02d%02d_%02d%02d%02d.psd", output_path, prefix, 
                rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second);
            // Open file to write raw data
	        if(psd_fp) fclose(psd_fp);
            //psd_fd = open(out_filename, (O_WRONLY | O_CREAT));
            psd_fp = fopen(out_filename, "w");
            fprintf(stdout, "Open psd file %s\n", out_filename);
		}    
	}
	  
    // Write waveform to output file
    if(hdr.type == WISPR_WAVEFORM) {
		if(wav_file == 1) {
			// write just the buffer data to wav file
			buffer_size = hdr.samples_per_block;
			nwrt = wav_write(dat_wf, buffer_data, buffer_size);
		}
		else if(dat_fp) {
			// write the entire buffer (header and data) to dat file
			buffer_size = (size_t)hdr.block_size;
			//nwrt = write(dat_fd, buffer, buffer_size);
			nwrt = fwrite(buffer, 1, buffer_size, dat_fp);
			if (nwrt =! buffer_size) {
				fprintf(stdout, "failed to write buffer: %d\n", nrd);
			}
		}
		if(dat_count++ == buffers_per_file) dat_count = 0;
	}
	
    // Write spectrum to output file
	if(hdr.type == WISPR_SPECTRUM) {
		if(wav_file == 1) {
			// write just the buffer data to wav file
			buffer_size = hdr.samples_per_block;
			nwrt = wav_write(psd_wf, buffer_data, buffer_size);
		} 
		else if(psd_fp) {
			// write the entire buffer (header and data) to dat file
			buffer_size = (size_t)hdr.block_size;
			//nwrt = write(psd_fd, buffer, buffer_size);
			nwrt = fwrite(buffer, 1, buffer_size, psd_fp);
			if (nwrt =! buffer_size) {
				fprintf(stdout, "failed to write psd buffer: %d\n", nrd);
			}
		}
		if(psd_count++ == buffers_per_file) psd_count = 0;
	}

  }

  if(dat_fp) fclose(dat_fp);
  if(psd_fp) fclose(psd_fp);
  if(psd_wf != NULL) wav_close(psd_wf);
  if(dat_wf != NULL) wav_close(dat_wf);
  
  fclose(fp);

  return 1;
}

uint32_t find_header(int fd) 
{
	uint32_t pos = 0;
	char buf, prev_buf;
	int nrd = 0; 
	while(1) {
		nrd = read(fd, &buf, 1); pos++;
		if(buf == 'W') {
			nrd = read(fd, &buf, 1); pos++;
			if(buf == 'I') {
				nrd = read(fd, &buf, 1); pos++;
				if(buf == 'S') {
					nrd = read(fd, &buf, 1); pos++;
					if(buf == 'P') {
						nrd = read(fd, &buf, 1); pos++;
						if(buf == 'R') {
							fprintf(stdout, "WISPR header found at %d\n", pos-5);
							return(pos-5);
						}
					}
				}
			}
		}
	}
	return(-1);
}


