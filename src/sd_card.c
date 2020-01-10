/**
 *
 */

#include <asf.h>
#include <string.h>

#include "sd_card.h"
#include "rtc_time.h"

/* IRQ priority for PIO (The lower the value, the greater the priority) */
#define IRQ_PRIOR_PIO 0

// just one slot for now - so always use slot zero
uint8_t sd_card_slot = 0;

sd_card_t sd_card_instance[MAX_NUMBER_SD_CARDS];

//
// select the sd card and disable the other.
// turn the card power on and sets the multiplexer switch
//
#define sd_startup_delay_msec  1

int sd_card_select(uint8_t card_num)
{
	if(card_num == 2) { // select card 2
		ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD2);
	} else if(card_num == 1) { // select card 1
		ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD1);
	} else {
		printf("Unknown SD number: select SD Card Failed\n\r");
		return(0);
	}
	//printf("Enabling SD Card %d: ", card_num);
	return(card_num);
}

int sd_card_enable(uint8_t card_num)
{
	if(card_num == 2) {
		ioport_set_pin_level(PIN_ENABLE_SD2, SD_ENABLE); // turn power on
	} else if(card_num == 1) {
		ioport_set_pin_level(PIN_ENABLE_SD1, SD_ENABLE); // turn power on
	} else {
		printf("Unknown SD number: enable SD Card Failed\n\r");
	}
	if(sd_startup_delay_msec) delay_ms(sd_startup_delay_msec);
	//printf("Enabling SD Card %d: ", card_num);
	return(card_num);
}

int sd_card_disable(uint8_t card_num)
{
	if(card_num == 2) {
		ioport_set_pin_level(PIN_ENABLE_SD2, SD_DISABLE); // turn power off
	} else if(card_num == 1) {
		ioport_set_pin_level(PIN_ENABLE_SD1, SD_DISABLE); // turn power off
	} else {
		printf("Unknown SD number: enable SD Card Failed\n\r");
	}
	//printf("Enabling SD Card %d: ", card_num);
	return(card_num);
}

/*
 */
int sd_card_open(uint8_t card_num, char *name, uint8_t reset_card)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(-1);
	}
	
	sd_card_t *card = &sd_card_instance[card_num];
	
	sd_card_enable(card_num);
	sd_card_select(card_num);
	
	sd_mmc_init();	

	card->state = sd_mmc_check(sd_card_slot);
	int count = 0;
	while (card->state != SD_MMC_OK  && count < 4) {
		card->state = sd_mmc_check(sd_card_slot);
		delay_ms(10);
		count++;
	}
	
	card->type = sd_mmc_get_type(sd_card_slot);
	card->version = sd_mmc_get_version(sd_card_slot);
	card->capacity = sd_mmc_get_capacity(sd_card_slot);  // capacity in KBytes
	
	// read the card header and check if its valid
	if( sd_card_read_header(card) == 0 || reset_card ) { 
		// so reset the card header
		card->start_block = SD_CARD_START_BLOCK;
		card->end_block = card->capacity * (1024 / SD_MMC_BLOCK_SIZE);
		// this will cause an overwrite of existing data
		card->write_addr = SD_CARD_START_BLOCK;
		card->read_addr = SD_CARD_START_BLOCK;
	}
	
	// always overwrite card name
	strncpy(card->name, name, 8);
	
	// set header mod time
	rtc_get_epoch( &card->epoch );
	
	// rewrite the header
	sd_card_write_header(card);

	if(card->state != SD_MMC_OK) {
		printf("SD Card Initialization failed, 0x%x\r\n", card->state);
	}
	
	return(card->state);
}

/*
 */
void sd_card_close(uint8_t card_num)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return;
	}
	sd_card_t *card = &sd_card_instance[card_num];

	// set header mod time
	rtc_get_epoch( &card->epoch );
	
	// rewrite the header
	sd_card_write_header(card);

	sd_card_disable(card_num);
}

void sd_card_print_info(uint8_t card_num)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
	}
	sd_card_t *card = &sd_card_instance[card_num];
	
	switch(card->type)
	{
		case CARD_TYPE_SD:
		printf("- Type: Normal SD\r\n");
		break;
		case (CARD_TYPE_HC | CARD_TYPE_SD):
		printf("- Type: SD High Capacity\r\n");
		break;
		default:
		printf("- Type: Unknown type %d, %d\r\n", card->type);
	}
	printf("- Name: %s:\r\n", card->name);
	printf("- Size: %lu KB\r\n", card->capacity);
	printf("- Version: %d\r\n", card->version);
	printf("- Start block: %d\r\n", card->start_block);
	printf("- End block: %d\r\n", card->end_block);
	printf("- Write Addr: %d\r\n", card->write_addr);
	printf("- Read Addr:  %d\r\n", card->read_addr);
	rtc_time_t tme;
	epoch_to_rtc_time(&tme, card->epoch);
	printf("- Last mod time: %02d/%02d/%02d %02d-%02d-%02d\r\n",
		tme.year,tme.month,tme.day,tme.hour,tme.minute,tme.second);
	
	//printf("- Last block written: %lu \n\r", hdr->last_block);
	//printf("    header size: %d\r\n", hdr->size);
	//printf("    start block addr: %lu \n\r", hdr->start_block);
	//printf("    end block addr: %lu \n\r", hdr->end_block);
}

sd_mmc_err_t sd_card_write(uint8_t card_num, uint8_t *buffer, uint16_t nblocks)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(-1);
	}
	sd_card_t *card = &sd_card_instance[card_num];
		
	sd_mmc_err_t ret = SD_MMC_OK;
	
	if(card->state != SD_MMC_OK) {
		printf("SD card NOT OK\r\n");
		return(card->state);
	}
	
	// current write address (in blocks)
	uint32_t addr = card->write_addr;
	
	// check address
	if( (addr < card->start_block) || (addr > (card->end_block - nblocks)) ) {
		printf("SD card is full: %d %d %d\r\n", addr, card->end_block, nblocks);
		return(SD_MMC_ERR_COMM);
	}

	// raw write to sd card
	ret = sd_card_write_raw(buffer, nblocks, addr);

	if( ret == SD_MMC_OK ) {
		// update the current write address
		card->write_addr += nblocks;
	}

	return(ret);
}

sd_mmc_err_t sd_card_read(uint8_t card_num, uint8_t *buffer, uint16_t nblocks)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(-1);
	}
	sd_card_t *card = &sd_card_instance[card_num];
	sd_mmc_err_t ret = SD_MMC_OK;
	
	if(card->state != SD_MMC_OK) {
		printf("SD card NOT OK:\r\n");
		return(card->state);
	}
	
	// current read address (in blocks)
	uint32_t addr = card->read_addr;
	
	// check address
	if( (addr < card->start_block) || (addr > (card->end_block - nblocks)) ) {
		printf("SD card NOT OK:\r\n");
		return(SD_MMC_ERR_COMM);
	}
	
	// raw write to sd card
	ret = sd_card_read_raw(buffer, nblocks, addr);

	if( ret == SD_MMC_OK ) {
		// update the current read address
		card->read_addr += nblocks;
	}

	return(ret);
}

sd_mmc_err_t sd_card_write_raw(uint8_t *buffer, uint16_t nblocks, uint32_t addr)
{
	sd_mmc_err_t ret = SD_MMC_OK;
	
	ret = sd_mmc_init_write_blocks(sd_card_slot, addr, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write: init_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	ret = sd_mmc_start_write_blocks(buffer, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write: start_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	ret = sd_mmc_wait_end_of_write_blocks(sd_card_slot);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write: wait_end_of_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	return(ret);
}


sd_mmc_err_t sd_card_read_raw(uint8_t *buffer, uint16_t nblocks, uint32_t addr)
{
	sd_mmc_err_t ret = 0;

	ret = sd_mmc_init_read_blocks(sd_card_slot, addr, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read_raw: init_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	ret = sd_mmc_start_read_blocks(buffer, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read_raw: start_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	ret = sd_mmc_wait_end_of_read_blocks(sd_card_slot);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read_raw: wait_end_of_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	return(SD_MMC_OK);
}


uint8_t sd_card_header_block[SD_MMC_BLOCK_SIZE];

sd_mmc_err_t sd_card_read_header(sd_card_t *hdr)
{
	sd_mmc_err_t ret = SD_MMC_OK;
	
	if(hdr->state != SD_MMC_OK) {
		printf("SD card NOT OK:\r\n");
		return(hdr->state);
	}
		
	// current write address (in blocks)
	uint32_t addr = SD_CARD_HEADER_BLOCK;
	
	// raw read to sd card
	ret = sd_card_read_raw(sd_card_header_block, 1, addr);
	if( ret != SD_MMC_OK ) {
		printf("sd_card_read_header: error %d\r\n", ret);
		return(ret);
	}

	if( sd_card_parse_header(sd_card_header_block, hdr) == 0 ) {
		printf("sd_card_read_header: card is not configured\r\n");
		return(0);
	}
	
	return(ret);
}

sd_mmc_err_t sd_card_write_header(sd_card_t *hdr)
{
	sd_mmc_err_t ret = SD_MMC_OK;
	
	if(hdr->state != SD_MMC_OK) {
		printf("SD card NOT OK:\r\n");
		return(hdr->state);
	}

	// current write address (in blocks)
	uint32_t addr = SD_CARD_HEADER_BLOCK;

	// unparse the config header into config block buffer
	sd_card_serialize_header(hdr, sd_card_header_block);
	
	// raw write to sd card
	ret = sd_card_write_raw(sd_card_header_block, 1, addr);
	
	return(ret);
}

//
//
//
int sd_card_parse_header(uint8_t *buf, sd_card_t *hdr)
{
	if ((buf[0] != 'W') && (buf[1] != 'I') && (buf[2] != 'S') && (buf[3] != 'P') && (buf[4] != 'R')) {
		fprintf(stdout, "wispr_parse_header: unrecognized\n");
		return(0);
	}

	//hdr->size = buf[5];
	hdr->version = buf[6];
	hdr->name[0] = buf[7]; 
	hdr->name[1] = buf[8]; 
	hdr->name[2] = buf[9];
	hdr->name[3] = buf[10];
	hdr->name[4] = buf[11];
	hdr->name[5] = buf[12];
	hdr->name[6] = buf[13];
	hdr->name[7] = buf[14];
	hdr->start_block = (uint32_t)buf[15] | ((uint32_t)buf[16] << 8) | ((uint32_t)buf[17] << 16) | ((uint32_t)buf[18] << 24);    // addr of first block (uint32_t)
	hdr->end_block = (uint32_t)buf[19] | ((uint32_t)buf[20] << 8) | ((uint32_t)buf[21] << 16) | ((uint32_t)buf[22] << 24);    // addr of last block (uint32_t)
	hdr->write_addr = (uint32_t)buf[23] | ((uint32_t)buf[24] << 8) | ((uint32_t)buf[25] << 16) | ((uint32_t)buf[26] << 24);    // addr of current write block (uint32_t)
	hdr->read_addr = (uint32_t)buf[25] | ((uint32_t)buf[26] << 8) | ((uint32_t)buf[27] << 16) | ((uint32_t)buf[28] << 24);    // addr of current write block (uint32_t)
	hdr->epoch = (uint32_t)buf[29] | ((uint32_t)buf[30] << 8) | ((uint32_t)buf[31] << 16) | ((uint32_t)buf[32] << 24);    // addr of current write block (uint32_t)
	//hdr->modtime.year = buf[27];
	//hdr->modtime.month = buf[28];
	//hdr->modtime.day = buf[29];
	//hdr->modtime.hour = buf[30];
	//hdr->modtime.minute = buf[31];
	//hdr->modtime.second = buf[32];
	return(31);
}

int sd_card_serialize_header(sd_card_t *hdr, uint8_t *buf)
{
	buf[0]   = 'W';
	buf[1]   = 'I';
	buf[2]   = 'S';
	buf[3]   = 'P';
	buf[4]   = 'R';

	buf[5]   = 35; // header size

	buf[6]   = (uint8_t)(hdr->version);

	buf[7]   = hdr->name[0];
	buf[8]   = hdr->name[1];
	buf[9]   = hdr->name[2];
	buf[10]  = hdr->name[3];
	buf[11]  = hdr->name[4];
	buf[12]  = hdr->name[5];
	buf[13]  = hdr->name[6];
	buf[14]  = hdr->name[7];

	buf[15]  = (uint8_t)(hdr->start_block >> 0);
	buf[16]  = (uint8_t)(hdr->start_block >> 8);
	buf[17]  = (uint8_t)(hdr->start_block >> 16);
	buf[18]  = (uint8_t)(hdr->start_block >> 24);
	buf[19]  = (uint8_t)(hdr->end_block >> 0);
	buf[20]  = (uint8_t)(hdr->end_block >> 8);
	buf[21]  = (uint8_t)(hdr->end_block >> 16);
	buf[22]  = (uint8_t)(hdr->end_block >> 24);

	buf[23]  = (uint8_t)(hdr->write_addr >> 0);
	buf[24]  = (uint8_t)(hdr->write_addr >> 8);
	buf[25]  = (uint8_t)(hdr->write_addr >> 16);
	buf[26]  = (uint8_t)(hdr->write_addr >> 24);
	buf[27]  = (uint8_t)(hdr->read_addr >> 0);
	buf[28]  = (uint8_t)(hdr->read_addr >> 8);
	buf[29]  = (uint8_t)(hdr->read_addr >> 16);
	buf[30]  = (uint8_t)(hdr->read_addr >> 24);

	buf[31]  = (uint8_t)(hdr->epoch >> 0);
	buf[32]  = (uint8_t)(hdr->epoch >> 8);
	buf[33]  = (uint8_t)(hdr->epoch >> 16);
	buf[34]  = (uint8_t)(hdr->epoch >> 24);

	//buf[27]  = (uint8_t)(hdr->modtime.year);
	//buf[28]  = (uint8_t)(hdr->modtime.month);
	//buf[29]  = (uint8_t)(hdr->modtime.day);
	//buf[30]  = (uint8_t)(hdr->modtime.hour);
	//buf[31]  = (uint8_t)(hdr->modtime.minute);
	//buf[32]  = (uint8_t)(hdr->modtime.second);

	return(35);
}


sd_mmc_err_t sd_card_read_config(uint8_t card_num, wispr_config_t *hdr)
{
	if( card_num < 0 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(-1);
	}
	sd_card_t *card = &sd_card_instance[card_num];
	sd_mmc_err_t ret = SD_MMC_OK;
	
	if(card->state != SD_MMC_OK) {
		printf("SD card NOT OK:\r\n");
		return(card->state);
	}
	
	// current write address (in blocks)
	uint32_t addr = SD_CARD_CONFIG_BLOCK;
	
	// raw read to sd card
	ret = sd_card_read_raw(sd_card_header_block, 1, addr);
	if( ret != SD_MMC_OK ) {
		printf("sd_card_read_header: error %d\r\n", ret);
		return(ret);
	}

	int m = wispr_parse_config(sd_card_header_block, hdr);
	if( m == 0 ) {
		printf("sd_card_read_header: card is not configured\r\n");
		return(SD_MMC_ERR_UNUSABLE);
	}
	
	return(ret);
}

sd_mmc_err_t sd_card_write_config(uint8_t card_num, wispr_config_t *hdr)
{
	if( card_num < 0 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(-1);
	}
	sd_card_t *card = &sd_card_instance[card_num];
	sd_mmc_err_t ret = SD_MMC_OK;
	
	if(card->state != SD_MMC_OK) {
		printf("SD card NOT OK:\r\n");
		return(card->state);
	}
	
	// current write address (in blocks)
	uint32_t addr = SD_CARD_CONFIG_BLOCK;

	// unparse the config header into config block buffer
	wispr_serialize_config(hdr, sd_card_header_block);
	
	// raw write to sd card
	ret = sd_card_write_raw(sd_card_header_block, 1, addr);
	
	return(ret);
}


int sd_card_is_full(uint8_t card_num, uint32_t nblocks)
{
   if(sd_card_instance[card_num].write_addr >= (sd_card_instance[card_num].end_block - nblocks)) return(1);
   else return(0);
}

int sd_card_set_number_of_blocks(uint8_t card_num, uint32_t nblocks)
{
	if( card_num < 0 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(-1);
	}
	sd_card_t *card = &sd_card_instance[card_num];
	
	uint32_t first = SD_CARD_START_BLOCK;
	uint32_t last = card->capacity * (1024 / SD_MMC_BLOCK_SIZE);
	if( (nblocks <= 0) || (nblocks > (last - first)) ) {
		printf("sd_card_set_number_of_blocks: failed\n\r");
		return(0);
	}
	card->end_block = card->start_block + nblocks;
	return(1);
}

uint32_t sd_card_get_number_of_blocks(uint8_t card_num)
{
	if( card_num < 0 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(-1);
	}
	sd_card_t *card = &sd_card_instance[card_num];
	return(card->end_block - card->start_block);
}


//-------------------------------------------------------------------------
//
// FAT Utils
//
#include <ff.h>
FATFS fat_fs;
FIL file_object;

int sd_card_init_fat(void)
{
	printf("Mount FAT on SD Card.\r\n");
	memset(&fat_fs, 0, sizeof(FATFS));
	FRESULT res = f_mount(LUN_ID_SD_MMC_0_MEM, &fat_fs);
	if (res != FR_OK) {
		printf("f_mount FAILED: res %d\r\n", res);
	}
	return(res);
}

FRESULT sd_card_open_file(char *filename)
{
	FRESULT res; 
	res = f_open(&file_object, (char const *)filename, FA_CREATE_ALWAYS | FA_WRITE);
	if( res != FR_OK ) {
		printf("f_open FAILED: res %d\r\n", res);
	}
	return(res);
}

int sd_card_write_file(uint8_t *buffer, uint16_t nblocks)
{
	UINT nbytes = nblocks * SD_MMC_BLOCK_SIZE;

	// write to file on SD card
	size_t nwrt = 0;
	FRESULT res = f_write(&file_object, buffer, nbytes, &nwrt);
	if( nwrt != nbytes ) {
		printf("Error writing to SD: %d, %d %d\r\n", res, nwrt, nblocks);
		return(0);
	}
	
	return(nwrt);
	
}


