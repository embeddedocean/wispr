/**
 * sd_card.c: sd card utility functions
 * 
 * There is no file system on the card, reads and writes access the card memory directly.
 * 
 Notes:
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
		sd_card_instance[1].state |= SD_CARD_SELECTED;
		sd_card_instance[0].state &= ~SD_CARD_SELECTED;
	} else if(card_num == 1) { // select card 1
		ioport_set_pin_level(PIN_SELECT_SD, SELECT_SD1);
		sd_card_instance[0].state |= SD_CARD_SELECTED;
		sd_card_instance[1].state &= ~SD_CARD_SELECTED;
	} else {
		printf("sd_card_select: unknown card number\n\r");
		return(0);
	}
	//printf("Enabling SD Card %d: ", card_num);
	return(1);
}

int sd_card_enable(uint8_t card_num)
{
	if(card_num == 2) {
		ioport_set_pin_level(PIN_ENABLE_SD2, SD_ENABLE); // turn power on
	} else if(card_num == 1) {
		ioport_set_pin_level(PIN_ENABLE_SD1, SD_ENABLE); // turn power on
	} else {
		printf("sd_card_enable: unknown card number\n\r");
		return(0);
	}
	if(sd_startup_delay_msec) delay_ms(sd_startup_delay_msec);
	sd_card_instance[card_num-1].state |= SD_CARD_ENABLED ;
	//printf("Enabling SD Card %d: ", card_num);
	return(1);
}

int sd_card_disable(uint8_t card_num)
{
	if(card_num == 2) {
		ioport_set_pin_level(PIN_ENABLE_SD2, SD_DISABLE); // turn power off
	} else if(card_num == 1) {
		ioport_set_pin_level(PIN_ENABLE_SD1, SD_DISABLE); // turn power off
	} else {
		printf("sd_card_disable: unknown card number\n\r");
		return(0);
	}
	sd_card_instance[card_num-1].state &= ~SD_CARD_ENABLED ;
	//printf("Enabling SD Card %d: ", card_num);
	return(1);
}


/*
 * sd_card_init enables, selects, and checks the sd card, leaving it in an active state
 */
uint8_t sd_card_init(uint8_t card_num)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD card number\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];

	// clear the card state
	//card->state &= !SD_CARD_OK;
	card->state = 0;

	sd_card_enable(card_num);
	sd_card_select(card_num);

	/* Initialize SD MMC stack */
	sd_mmc_init();

	sd_mmc_err_t err = sd_mmc_check(sd_card_slot);
	int count = 0;
	while (err != SD_MMC_OK  && count < 4) {
		err = sd_mmc_check(sd_card_slot);
		delay_ms(10);
		count++;
	}
	if( err != SD_MMC_OK ) {
		printf("sd_card_init: card check error\n\r");
		return( card->state );
	}
	
	card->state |= SD_CARD_OK;
	card->number = card_num;
	card->type = sd_mmc_get_type(sd_card_slot);
	card->hw_ver = sd_mmc_get_version(sd_card_slot);
	card->capacity = sd_mmc_get_capacity(sd_card_slot);  // capacity in KBytes

	card->version[0] = WISPR_VERSION;
	card->version[1] = WISPR_SUBVERSION;
	
	return( card->state );
}

/*
 */
uint8_t sd_card_format(uint8_t card_num, char *name)
{
	uint8_t state = sd_card_init(card_num);
	if( !(state & SD_CARD_OK) ) {
		printf("sd_card_format: card init error\n\r");
		return( state );
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	// reset the card header block info
	card->start_block = SD_CARD_START_BLOCK;
	card->end_block = card->capacity * (1024 / SD_MMC_BLOCK_SIZE) - 1;
	// this will cause an overwrite of existing data
	card->write_addr = SD_CARD_START_BLOCK;
	card->read_addr = SD_CARD_START_BLOCK;
	
	// always overwrite card name
	strncpy(card->name, name, 8);

	// set header mod time
	rtc_get_epoch( &card->epoch );

	// rewrite the header
	sd_card_write_header(card);

	// note - the state is not written to the sd card header block
	card->state |= SD_CARD_FORMATED;

	return( card->state );
}

//
// sd_card_open - initializes the card and reads the card header.
// The card header contains the most recent read/write address pointers.
// If the card header is not formatted (no header present), it sets the state as unformatted
// but leaves the card in an open state.
//
uint8_t sd_card_open(uint8_t card_num)
{
	uint8_t state = sd_card_init(card_num);
	if( !(state & SD_CARD_OK) ) {
		printf("sd_card_open: card init error\n\r");
		return( state );
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	// read the card header and check if its valid
	if( (sd_card_read_header(card) == 0) ) {
		card->state &= ~SD_CARD_FORMATED;
	} else {
		card->state |= SD_CARD_FORMATED;		
	}
	
	// set header mod time
	rtc_get_epoch( &card->epoch );
	
	card->state |= SD_CARD_OPEN;
	
	return( card->state );
}


/*
 * sd_card_close - updates the card header and disables the card (powering off).
 * closing does not un-select the card. 
 */
void sd_card_close(uint8_t card_num)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("sd_card_close: unknown card number\n\r");
		return;
	}
	sd_card_t *card = &sd_card_instance[card_num-1];

	if( !(card->state & SD_CARD_OPEN) ) {
		return;
	}

	// set header mod time
	rtc_get_epoch( &card->epoch );

	card->state &= ~(SD_CARD_OPEN);
	
	// update the header
	sd_card_write_header(card);

	// turn card power off
	sd_card_disable(card_num);
}

void sd_card_print_info(uint8_t card_num)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("sd_card_print_info: unknown card number\n\r");
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	printf("\n\rSD Card %d\n\r", card->number);

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
	printf("- Version: %d.%d\r\n", card->version[0], card->version[1]);
	printf("- Size: %lu KB\r\n", card->capacity);
	printf("- Start block: %d\r\n", card->start_block);
	printf("- End block: %d\r\n", card->end_block);
	printf("- Write Addr: %d\r\n", card->write_addr);
	printf("- Read Addr:  %d\r\n", card->read_addr);
	//printf("- State: %02x\r\n", card->state);
	rtc_time_t tme;
	epoch_to_rtc_time(&tme, card->epoch);
	printf("- Last mod time: %02d/%02d/%02d %02d-%02d-%02d (%lu)\r\n",
		tme.year,tme.month,tme.day,tme.hour,tme.minute,tme.second, card->epoch);
}

uint8_t sd_card_state(uint8_t card_num, uint8_t state)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("sd_card_state: unknown card number\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];	
	return(card->state & state);
}

//
// Check if card will be full. 
// Returns 1 if there is not enough space to write nblocks,
// otherwise returns 0.
int sd_card_is_full(uint8_t card_num, uint32_t nblocks)
{
	sd_card_t *card = &sd_card_instance[card_num-1];
	if( card->write_addr > (card->end_block - nblocks) ) {
		return(1);
	}
	return(0);
}


uint16_t sd_card_write(uint8_t card_num, uint8_t *buffer, uint16_t nblocks)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("sd_card_write: unknown card number\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	if( !(card->state & SD_CARD_READY) ) {
		printf("sd_card_write: card not open\n\r");
		return(0);
	}
	
	// current write address (in blocks)
	uint32_t addr = card->write_addr;
	
	// check address
	if( (addr < card->start_block) || (addr > (card->end_block - nblocks)) ) {
		printf("sd_card_write: address out or valid range\r\n");
		//printf("sd_card_write: card is full: %d %d %d\r\n", addr, card->end_block, nblocks);
		return(0);
	}

	// raw write to sd card
	if( sd_card_write_raw(buffer, nblocks, addr) != SD_MMC_OK ) {
		printf("sd_card_write: raw write error\r\n");
		return(0);
	}

	// update the current write address
	card->write_addr += nblocks;

	// update the card header
	sd_card_write_header(card);
	
	return( nblocks );
}

uint16_t sd_card_read(uint8_t card_num, uint8_t *buffer, uint16_t nblocks)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("sd_card_read: unknown card number\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];

	if( !(card->state & SD_CARD_READY) ) {
		printf("sd_card_read: card not open\n\r");
		return(0);
	}
	
	// current read address (in blocks)
	uint32_t addr = card->read_addr;
	
	// check address
	if( (addr < card->start_block) || (addr > (card->end_block - nblocks)) ) {
		printf("sd_card_read: invalid address\r\n");
		return(0);
	}
	
	// raw write to sd card
	if( sd_card_read_raw(buffer, nblocks, addr) != SD_MMC_OK ) {
		printf("sd_card_read: raw read error\r\n");
		return(0);
	}

	// update the current read address
	card->read_addr += nblocks;

	// update the card header
	sd_card_write_header(card);

	return( nblocks );
}

sd_mmc_err_t sd_card_write_raw(uint8_t *buffer, uint16_t nblocks, uint32_t addr)
{
	sd_mmc_err_t ret = SD_MMC_OK;
	
	ret = sd_mmc_init_write_blocks(sd_card_slot, addr, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write_raw: init_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	ret = sd_mmc_start_write_blocks(buffer, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write_raw: start_write_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	ret = sd_mmc_wait_end_of_write_blocks(sd_card_slot);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_write_raw: wait_end_of_write_blocks FAILED, %d\n\r", ret);
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

int sd_card_read_header(sd_card_t *hdr)
{
	// current write address (in blocks)
	uint32_t addr = SD_CARD_HEADER_BLOCK;
	
	// raw read to sd card
	if( sd_card_read_raw(sd_card_header_block, 1, addr) != SD_MMC_OK ) {
		printf("sd_card_read_header: raw read error\r\n");
		return(0);
	}

	int nrd = wispr_sd_card_parse_header(sd_card_header_block, hdr);
	if( nrd == 0 ) {
		printf("sd_card_read_header: card is not formatted\r\n");
	}	
	return(nrd);
}

int sd_card_write_header(sd_card_t *hdr)
{	
	// current write address (in blocks)
	uint32_t addr = SD_CARD_HEADER_BLOCK;

	// unparse the config header into config block buffer
	int nwrt = wispr_sd_card_serialize_header(hdr, sd_card_header_block);
	
	// raw write to sd card
	if( sd_card_write_raw(sd_card_header_block, 1, addr) != SD_MMC_OK) {
		printf("sd_card_write_header: raw write error\r\n");
		return(0);
	}
	return(nwrt);
}

uint8_t sd_card_config_block[SD_MMC_BLOCK_SIZE];

int sd_card_read_config(uint8_t card_num, wispr_config_t *hdr)
{
	if( card_num < 0 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(0);
	}
	//sd_card_t *card = &sd_card_instance[card_num-1];
	
	// current write address (in blocks)
	uint32_t addr = SD_CARD_CONFIG_BLOCK;
	
	// raw read to sd card
	if( sd_card_read_raw(sd_card_config_block, 1, addr) != SD_MMC_OK ) {
		printf("sd_card_read_config: raw read error\r\n");
		return(0);
	}

	int nrd = wispr_parse_config(sd_card_config_block, hdr);
	if( nrd == 0 ) {
		printf("sd_card_read_config: card is not configured\r\n");
		return(0);
	}
	return(nrd);
}

int sd_card_write_config(uint8_t card_num, wispr_config_t *hdr)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(0);
	}
	//sd_card_t *card = &sd_card_instance[card_num-1];
		
	// current write address (in blocks)
	uint32_t addr = SD_CARD_CONFIG_BLOCK;

	// unparse the config header into config block buffer
	int nwrt = wispr_serialize_config(hdr, sd_card_config_block);
	
	// raw write to sd card
	if( sd_card_write_raw(sd_card_config_block, 1, addr) != SD_MMC_OK) {
		printf("sd_card_write_config: raw write error\r\n");
		return(0);
	}
	return(nwrt);
}

int sd_card_set_number_of_blocks(uint8_t card_num, uint32_t nblocks)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];

	if( !(card->state & SD_CARD_OPEN) ) {
		printf("sd_card_set_number_of_blocks: card not open\n\r");
		return(0);
	}
	
	uint32_t first = SD_CARD_START_BLOCK;
	uint32_t last = card->capacity * (1024 / SD_MMC_BLOCK_SIZE);
	if( (nblocks <= 0) || (nblocks > (last - first)) ) {
		printf("sd_card_set_number_of_blocks: failed to set %d\n\r", nblocks);
		return(0);
	}
	card->end_block = card->start_block + nblocks;

	// rewrite the header
	sd_card_write_header(card);

	return(1);
}

uint32_t sd_card_get_number_of_blocks(uint8_t card_num)
{
	if( card_num < 1 || card_num > MAX_NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
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


