/**
 * sd_card_lite.h
 */
#ifndef _SD_CARD_H_
#define _SD_CARD_H_

#include <string.h>

#include "wispr.h"
#include "rtc_time.h"

#define NUMBER_SD_CARDS 2

#define SD_CARD_BLOCK_SIZE    WISPR_SD_CARD_BLOCK_SIZE 
#define SD_CARD_HEADER_BLOCK  WISPR_SD_CARD_HEADER_BLOCK
#define SD_CARD_CONFIG_BLOCK  WISPR_SD_CARD_CONFIG_BLOCK
#define SD_CARD_START_BLOCK   WISPR_SD_CARD_START_BLOCK

#define SD_CARD_HEADER_VERSION 1

// card states
#define SD_CARD_OK	      0x01
#define SD_CARD_FORMATED  0x02
#define SD_CARD_OPEN      0x04
#define SD_CARD_FULL      0x08
#define SD_CARD_ENABLED   0x10
#define SD_CARD_SELECTED  0x20

// file states
#define SD_FILE_OPEN	  0x40
#define SD_FILE_FULL      0x80

// file systems types
#define SD_FS_RAW	  0x01
#define SD_FS_FAT	  0x02

// for a card to be ready for read/write it nit have all these states set
#define SD_CARD_READY  (SD_CARD_OK|SD_CARD_FORMATED|SD_CARD_OPEN|SD_CARD_ENABLED|SD_CARD_SELECTED)

typedef struct {
	char     label[16];     // user set identifier
	uint8_t  fs;            // file system type (0=non, 1=exFat)
	uint8_t  number;        // card number
	uint8_t  version[2];    // wispr version
	uint8_t  hw_ver;        // card hardware version
	uint8_t  state;         // current state (OPEN, ENABLED, ..)
	uint8_t  type;          // hardware type
	uint32_t capacity;      // total card capacity in KBytes
	uint32_t start;         // addr of start block
	uint32_t end;           // addr of end block
	uint32_t write_addr;    // addr of current write block
	uint32_t read_addr;     // addr of current read block
	uint32_t total;         // total number of sectors KBytes
	uint32_t free;          // available sectors in KBytes
	uint32_t epoch;         // mod time
} sd_card_t;


extern void sd_card_print_header(sd_card_t *hdr);
extern int sd_card_parse_header(uint8_t *buf, sd_card_t *hdr);
extern int sd_card_serialize_header(sd_card_t *hdr, uint8_t *buf);


#endif
