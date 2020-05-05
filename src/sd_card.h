/**
 * sd_card.h
 */
#ifndef _SD_CARD_H_
#define _SD_CARD_H_

#include <asf.h>
#include <string.h>

//#include "conf_sd_mmc.h"
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
#define SD_FILE_CLOSED	  0x00
#define SD_FILE_OPEN	  0x01
#define SD_FILE_FULL      0x02

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

typedef struct {
	char name[32];    // user set identifier
	rtc_time_t time;  // time open
	uint8_t  state;
	uint32_t count;   // current block count
	uint32_t size;    // size in blocks
	FIL file;
	uint8_t card_num;
} fat_file_t;

//typedef wispr_sd_card_t sd_card_t;

extern int sd_card_select(uint8_t card_num);
extern int sd_card_enable(uint8_t card_num);
extern int sd_card_disable(uint8_t card_num);
extern uint8_t sd_card_init(uint8_t card_num);

extern void sd_card_print_info(uint8_t card_num);
extern uint8_t sd_card_state(uint8_t card_num, uint8_t state);
extern int sd_card_set_number_of_blocks(uint8_t card_num, uint32_t nblocks);
extern uint32_t sd_card_get_number_of_blocks(uint8_t card_num);
extern uint32_t sd_card_get_epoch(uint8_t card_num);

extern int sd_card_is_full(uint8_t card_num, uint32_t nblocks);
extern uint32_t sd_card_free_space(uint8_t card_num);

extern void sd_card_print_header(sd_card_t *hdr);
extern int sd_card_parse_header(uint8_t *buf, sd_card_t *hdr);
extern int sd_card_serialize_header(sd_card_t *hdr, uint8_t *buf);

extern uint8_t sd_card_format(uint8_t card_num, char *name);
extern uint8_t sd_card_open(uint8_t card_num);
extern void sd_card_close(uint8_t card_num);
extern uint16_t sd_card_write(uint8_t card_num, uint8_t *buffer, uint16_t nblocks);
extern uint16_t sd_card_read(uint8_t card_num, uint8_t *buffer, uint16_t nblocks);
extern int sd_card_write_header(sd_card_t *sd_card);
extern int sd_card_read_header(sd_card_t *sd_card);
extern int sd_card_write_config(uint8_t card_num, wispr_config_t *hdr);
extern int sd_card_read_config(uint8_t card_num, wispr_config_t *hdr);

extern FRESULT sd_card_format_fat(uint8_t card_num);
extern FRESULT sd_card_mount_fat(uint8_t card_num);
extern FRESULT sd_card_umount_fat(uint8_t card_num);
extern FRESULT sd_card_open_fat(fat_file_t *ff, char *name, unsigned char mode, uint8_t card_num);
extern FRESULT sd_card_close_fat(fat_file_t *ff);
extern FRESULT sd_card_write_fat(fat_file_t *ff, uint8_t *buffer, uint16_t nblocks);
extern FRESULT sd_card_write_config_fat(char *filename, wispr_config_t *hdr);
extern FRESULT sd_card_read_config_fat(char *filename, wispr_config_t *hdr);
extern FRESULT sd_card_set_fat_file_size(fat_file_t *ff, uint32_t size);


#endif
