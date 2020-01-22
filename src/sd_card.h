/**
 * sd_card.h
 */
#ifndef _SD_CARD_H_
#define _SD_CARD_H_

#include <asf.h>
#include <string.h>

#include "conf_sd_mmc.h"

#include "wispr.h"
#include "rtc_time.h"

#define MAX_NUMBER_SD_CARDS 2

#define SD_CARD_BLOCK_SIZE WISPR_SD_CARD_BLOCK_SIZE 
#define SD_CARD_HEADER_BLOCK WISPR_SD_CARD_HEADER_BLOCK
#define SD_CARD_CONFIG_BLOCK WISPR_SD_CARD_CONFIG_BLOCK
#define SD_CARD_START_BLOCK WISPR_SD_CARD_START_BLOCK

#define SD_CARD_HEADER_VERSION 1

// card states
#define SD_CARD_OK		 0x00
#define SD_CARD_OPEN     0x01
#define SD_CARD_ENABLED  0x02
#define SD_CARD_SELECTED 0x04
#define SD_CARD_FULL     0x08

//typedef struct {	
//	char     type[5];      // fixed identifier
//	uint8_t  size;         // size in bytes of the header record
//	uint8_t  version;      // version number
//	char     name[8];      // user set identifier
//	uint32_t start_block;  // addr of start block
//	uint32_t end_block;    // addr of end block
//	uint32_t last_block;   // addr of last block written
//	rtc_time_t modtime;    // time last modified
//} sd_card_header_t;

typedef wispr_sd_card_t sd_card_t;

extern int sd_card_select(uint8_t card_num);
extern int sd_card_enable(uint8_t card_num);
extern int sd_card_disable(uint8_t card_num);

extern int sd_card_init(uint8_t card_num, char *name, uint8_t reset_card);
extern int sd_card_open(uint8_t card_num);
extern void sd_card_close(uint8_t card_num);
extern void sd_card_print_info(uint8_t card_num);
extern uint8_t sd_card_state(uint8_t card_num, uint8_t state);

extern uint16_t sd_card_write(uint8_t card_num, uint8_t *buffer, uint16_t nblocks);
extern uint16_t sd_card_read(uint8_t card_num, uint8_t *buffer, uint16_t nblocks);

extern sd_mmc_err_t sd_card_write_raw(uint8_t *buffer, uint16_t nblocks, uint32_t addr);
extern sd_mmc_err_t sd_card_read_raw(uint8_t *buffer, uint16_t nblocks, uint32_t addr);

extern int sd_card_write_header(sd_card_t *sd_card);
extern int sd_card_read_header(sd_card_t *sd_card);

//extern int sd_card_parse_header(uint8_t *buf, sd_card_t *hdr);
//extern int sd_card_serialize_header(sd_card_t *hdr, uint8_t *buf);

extern int sd_card_write_config(uint8_t card_num, wispr_config_t *hdr);
extern int sd_card_read_config(uint8_t card_num, wispr_config_t *hdr);

extern int sd_card_is_full(uint8_t card_num, uint32_t nblocks);
extern int sd_card_set_number_of_blocks(uint8_t card_num, uint32_t nblocks);
extern uint32_t sd_card_get_number_of_blocks(uint8_t card_num);

extern int sd_card_init_fat(void);
extern FRESULT sd_card_open_file(char *filename);
extern int sd_card_write_file(uint8_t *buffer, uint16_t nblocks);

#endif