/*
 * read_sdcard.c
 *
 */
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

// This is the first block on the SD card where data exists
#define SD_CARD_START_BLOCK         2072U
#define SD_CARD_BLOCK_SIZE          512U

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
  char *input_path = "/dev/sdb";
  char *output_path = ".";
  char *prefix = "WISPR";

  char out_filename[128];
  int c = 0;
  int buffer_count = 0;
  int buffers_per_file = 50;
  int buffer_size = 0;
  int input_fd;
  size_t nrd = 0;
  size_t nwrt = 0;
  size_t start_block = WISPR_SD_CARD_START_BLOCK;
  int wave_file = 0;
  uint32_t pos = 0;

  int dat_fd = -1;
  int psd_fd = -1;

  // Parse command line args
  while ((c = getopt(argc, argv, "b:s:i:o:p:h")) != -1) 
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
        wave_file = 1;
        break;
      }
      case 'h':
      {
        fprintf(stdout, "Read SD Card and store data to file\n");
        fprintf(stdout, "  Optional command line arguments\n");
        fprintf(stdout, "    -i - Specify the input path raw data.  Defaults = /dev/sdb\n");
        fprintf(stdout, "    -o - Specify the path to save data files. Default = /mnt/rockhopper\n");
        fprintf(stdout, "Usage %s [-i /input/dev/path -o /output/dev/path]\n", argv[0]);
        return 1;
      }
    }
  }
  
  // Open the device with raw samples
  //input_fd = open(input_path, (O_RDONLY | O_LARGEFILE));
  input_fd = open(input_path, (O_RDONLY ));
  
  if(input_fd < 0) {
    fprintf(stdout, "failed to open SD Card: %d\n", input_fd);
    return -1;
  }
  fprintf(stdout, "Openned SD Card\n");

  // read the card header block
  pos = (WISPR_SD_CARD_BLOCK_SIZE * WISPR_SD_CARD_HEADER_BLOCK);
  lseek(input_fd, pos, SEEK_SET);
  nrd = read(input_fd, buffer, WISPR_SD_CARD_BLOCK_SIZE);
  if( wispr_sd_card_parse_header(buffer, &sd) == 0 ) {
     fprintf(stdout, "failed to parse card header\n");
  }

  // read the configuration block of the card
  pos = (WISPR_SD_CARD_BLOCK_SIZE * WISPR_SD_CARD_CONFIG_BLOCK);
  lseek(input_fd, pos, SEEK_SET);
  nrd = read(input_fd, buffer, WISPR_SD_CARD_BLOCK_SIZE);
  if( wispr_parse_config(buffer, &cfg) == 0 ) {
     fprintf(stdout, "failed to parse configuration\n");
  }

  wispr_print_config(&cfg);
  
  //pos = find_header(input_fd);
  start_block = WISPR_SD_CARD_START_BLOCK;
  pos = (WISPR_SD_CARD_BLOCK_SIZE * start_block);

  // seek to start of data	  
  lseek(input_fd, pos, SEEK_SET);
  fprintf(stdout, "Seek to file position %d (block %d)\n", pos, start_block);


  int go = 1;
  while(go) 
  {

    // Read data buffer header
    nrd = read(input_fd, buffer_header, WISPR_DATA_HEADER_SIZE);
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
    nrd = read(input_fd, buffer_data, buffer_size);
    if (nrd =! buffer_size) {
       fprintf(stdout, "failed to read data block: %d\n", nrd);
       return -1;
    }
	
	// open new output file
    if(buffer_count == 0) {

	   wispr_print_data_header(&hdr);
		
       rtc_time_t rtc;
       epoch_to_rtc_time(&rtc, hdr.second);
       
	   if(hdr.type == WISPR_WAVEFORM) {
	      sprintf(out_filename,"%s/%s_%02d%02d%02d_%02d%02d%02d.dat", output_path, prefix, 
             rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second);
          // Open file to write raw data
	      if(dat_fd > 0) close(dat_fd);
          dat_fd = open(out_filename, (O_WRONLY | O_CREAT));
          fprintf(stdout, "Open output file %s\n", out_filename);
          if(dat_fd < 0) {
             fprintf(stdout, "failed to open output file: %d\n", dat_fd);
             return -1;
          }
	   }
	   else if(hdr.type == WISPR_SPECTRUM) {
	      sprintf(out_filename,"%s/%s_%02d%02d%02d_%02d%02d%02d.psd", output_path, prefix, 
             rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute, rtc.second);
          // Open file to write raw data
	      if(psd_fd > 0) close(psd_fd);
          psd_fd = open(out_filename, (O_WRONLY | O_CREAT));
          fprintf(stdout, "Open output file %s\n", out_filename);
          if(psd_fd < 0) {
             fprintf(stdout, "failed to open output file: %d\n", psd_fd);
             return -1;
          }
	   }
    
	}
	  
    // Wriet buffer to output file
    if(hdr.type == WISPR_WAVEFORM) {
	   if(dat_fd > 0) {
          buffer_size = (size_t)hdr.block_size;
          nwrt = write(dat_fd, buffer, buffer_size);
          if (nwrt =! buffer_size) {
             fprintf(stdout, "failed to write buffer: %d\n", nrd);
             //continue;
          }
	   }
	}
	else if(hdr.type == WISPR_SPECTRUM) {
	   if(psd_fd > 0) {
          buffer_size = (size_t)hdr.block_size;
          nwrt = write(psd_fd, buffer, buffer_size);
          if (nwrt =! buffer_size) {
             fprintf(stdout, "failed to write psd buffer: %d\n", nrd);
          }
	   }
	}

	// reset count
    if(buffer_count++ == buffers_per_file) buffer_count = 0;

  }

  close(dat_fd);
  close(psd_fd);
  close(input_fd);

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


