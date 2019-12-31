/**
 *
 */


#include <asf.h>
#include <string.h>

#include "sd_card.h"

/* IRQ priority for PIO (The lower the value, the greater the priority) */
#define IRQ_PRIOR_PIO    0

/*
 */
int sd_card_init(sd_card_t *sd_card)
{
	uint8_t slot = 0;  
	
	//hsmci_init();
	sd_mmc_init();	

	sd_card->state = sd_mmc_check(slot);
	int count = 0;
	while (sd_card->state != SD_MMC_OK  && count < 4) {
		sd_card->state = sd_mmc_check(slot);
		delay_ms(10);
		count++;
	}
	
	sd_card->slot = slot;
	sd_card->block_size = SD_MMC_BLOCK_SIZE;

	sd_card->type = sd_mmc_get_type(slot);
	sd_card->version = sd_mmc_get_version(slot);
	sd_card->capacity = sd_mmc_get_capacity(slot);  // capacity in KBytes
	
	sd_card->first_block = SD_CARD_START_BLOCK;
	sd_card->last_block = sd_card->capacity * (1024 / SD_MMC_BLOCK_SIZE);

	sd_card->write_addr = SD_CARD_START_BLOCK;
	sd_card->read_addr = SD_CARD_START_BLOCK;

	if(sd_card->state != SD_MMC_OK) {
		printf("SD Card Initialization failed, 0x%x\r\n", sd_card->state);
	}
	
	return(sd_card->state);
}

void sd_card_print_info(sd_card_t *sd_card)
{
	switch(sd_card->type)
	{
		case CARD_TYPE_SD:
		printf("- Type: Normal SD\r\n");
		break;
		case (CARD_TYPE_HC | CARD_TYPE_SD):
		printf("- Type: SD High Capacity\r\n");
		break;
		default:
		printf("- Type: Unknown type %d, %d\r\n", sd_card->type);
	}
	printf("- Total size: %lu KB\r\n", sd_card->capacity);
	printf("- Version: %d\r\n", sd_card->version);
	printf("- First block: %d\r\n", sd_card->first_block);
	printf("- Last  block: %d\r\n", sd_card->last_block);
	
}

int sd_card_write_raw(sd_card_t *sd_card, uint8_t *buffer, uint16_t nblocks)
{
	sd_mmc_err_t ret = 0;
	
	// current write address (in blocks)
	uint32_t addr = sd_card->write_addr;
	
	if(sd_card->state != SD_MMC_OK) {
		printf("SD card NOT OK:\r\n");
		return(sd_card->state);
	}
	
	ret = sd_mmc_init_write_blocks(sd_card->slot, addr, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write: init_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	ret = sd_mmc_start_write_blocks(buffer, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write: start_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	ret = sd_mmc_wait_end_of_write_blocks(sd_card->slot);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write: wait_end_of_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	// update the current write address
	sd_card->write_addr += nblocks;

	return(SD_MMC_OK);
	
}


int sd_card_read_raw(sd_card_t *sd_card, uint8_t *buffer, uint16_t nblocks)
{
	sd_mmc_err_t ret = 0;

	// current read address (in blocks)
	uint32_t addr = sd_card->read_addr;
	
	if(sd_card->state != SD_MMC_OK) {
		printf("SD card NOT OK:\r\n");
		return(sd_card->state);
	}
	
	ret = sd_mmc_init_read_blocks(sd_card->slot, addr, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read_raw: init_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	ret = sd_mmc_start_read_blocks(buffer, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read_raw: start_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	ret = sd_mmc_wait_end_of_read_blocks(sd_card->slot);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read_raw: wait_end_of_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	// update the current read address
	sd_card->read_addr += nblocks;

	return(SD_MMC_OK);
	
}

//
// select the sd card and disable the other
//
#define sd_startup_delay_msec  1
int sd_card_select(uint8_t card_num)
{
	if(card_num == 2) {
		// select and enable SD2
		ioport_set_pin_level(PIN_ENABLE_SD2, SD_ENABLE);
		ioport_set_pin_level(PIN_ENABLE_SD1, SD_DISABLE);
		if(sd_startup_delay_msec) delay_ms(sd_startup_delay_msec);
		ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD2);
	} else if(card_num == 1) {
		// select and enable SD1
		ioport_set_pin_level(PIN_ENABLE_SD1, SD_ENABLE);
		ioport_set_pin_level(PIN_ENABLE_SD2, SD_DISABLE);
		if(sd_startup_delay_msec) delay_ms(sd_startup_delay_msec);
		ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD1);
	} else {
		printf("Unknown SD number: select SD Card Failed\n\r");
		return(0);
	}
	//printf("Enabling SD Card %d: ", card_num);
	return(card_num);
}

int sd_card_is_full(sd_card_t *sd_card)
{
   if(sd_card->write_addr >= sd_card->last_block) return(1);
   else return(0);
}

int sd_card_set_last_block(sd_card_t *sd_card, uint32_t block)
{
	if(block <= sd_card->first_block ) {
		printf("sd_card_set_last_block: failed\n\r");
		return(0);
	}
	sd_card->last_block = block;
	return(1);
}

int sd_card_set_number_of_blocks(sd_card_t *sd_card, uint32_t nblocks)
{
	uint32_t first = SD_CARD_START_BLOCK;
	uint32_t last = sd_card->capacity * (1024 / SD_MMC_BLOCK_SIZE);
	if( (nblocks <= 0) || (nblocks > (last - first)) ) {
		printf("sd_card_set_number_of_blocks: failed\n\r");
		return(0);
	}
	sd_card->last_block = sd_card->first_block + nblocks;
	return(1);
}

uint32_t sd_card_get_number_of_blocks(sd_card_t *sd_card)
{
	return(sd_card->last_block - sd_card->first_block);
}

/*
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
*/

