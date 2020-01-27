/*
 * erase_sdcard.c
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

uint8_t buffer[WISPR_SD_CARD_BLOCK_SIZE];

int main(int argc, char **argv)
{
  char *input_path = "/dev/sdb";
  int buffer_count = 0;
  uint32_t pos = 0;
  int fd = -1;
  int c, n, nwrt;
  int start_block = 30;
  int end_block = 32;
  
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
      case 'e':
      {
        end_block = atoi(optarg);
        break;
      }
      case 'i':
      {
        input_path = optarg;
        break;
      }
      case 'h':
      {
        fprintf(stdout, "Erase SD Card\n");
        fprintf(stdout, "  Optional command line arguments\n");
        fprintf(stdout, "    -i - Specify the input path.  Defaults = /dev/sdb\n");
        fprintf(stdout, "    -s - Start block.  Defaults = 0\n");
        fprintf(stdout, "    -e - End block. Default = 0\n");
        return 1;
      }
    }
  }
  
  // Open the device with raw samples
  //input_fd = open(input_path, (O_RDONLY | O_LARGEFILE));
  fd = open(input_path, (O_WRONLY ));
  
  if(fd < 0) {
    fprintf(stdout, "failed to open SD Card: %d\n", fd);
    return -1;
  }
  fprintf(stdout, "Openned SD Card\n");

  
  pos = (WISPR_SD_CARD_BLOCK_SIZE * start_block);

  // seek to start of data	  
  lseek(fd, pos, SEEK_SET);
  fprintf(stdout, "Seek to file position %d (block %d)\n", pos, start_block);

  for(n=0; n < WISPR_SD_CARD_BLOCK_SIZE; n++) buffer[n] = 0;

  for(n = start_block; n < end_block; n++) {

    // Read data buffer header
    nwrt = write(fd, buffer, WISPR_SD_CARD_BLOCK_SIZE);
    if (nwrt =! WISPR_SD_CARD_BLOCK_SIZE) {
       fprintf(stdout, "failed to write: %d\n", nwrt);
       return -1;
    }

	pos += WISPR_SD_CARD_BLOCK_SIZE;

  }

  close(fd);

  return 1;
}

