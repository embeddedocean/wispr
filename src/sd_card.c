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

sd_card_t sd_card_instance[NUMBER_SD_CARDS];

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
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
		printf("Unknown SD card number\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];

	// clear the card state
	//card->state &= !SD_CARD_OK;
	card->state = 0;

	// hardware controls
	sd_card_enable(card_num);
	sd_card_select(card_num);

	/* Initialize SD MMC stack */
	sd_mmc_init();

	Ctrl_status status;
	do {
		status = sd_mmc_test_unit_ready(0);
		if (CTRL_FAIL == status) {
			printf("Card install FAIL\n\r");
			printf("Please unplug and re-plug the card.\n\r");
			while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
			}
		}
	} while (CTRL_GOOD != status);

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

void sd_card_print_info(uint8_t card_num)
{
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
		printf("sd_card_print_info: unknown card number\n\r");
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	printf("SD Card %d\n\r", card->number);

	switch(card->type) {
	 case CARD_TYPE_SD:
		printf("- Type: Normal SD\r\n");
		break;
	 case (CARD_TYPE_HC | CARD_TYPE_SD):
		printf("- Type: SD High Capacity\r\n");
		break;
	 default:
		printf("- Type: Unknown type %d, %d\r\n", card->type);
	}
	
	printf("- Label: %s:\r\n", card->label);

	switch(card->fs) {
	 case SD_FS_RAW:
		// no file system
		printf("- File system: Non\r\n");
		printf("- Start block: %d\r\n", card->start);
		printf("- End block:   %d\r\n", card->end);
		printf("- Write Addr:  %d\r\n", card->write_addr);
		printf("- Read  Addr:  %d\r\n", card->read_addr);
		break;
	 case SD_FS_FAT:
		// fat fs info
		printf("- File system:   FAT\r\n");
		printf("- Total sectors: %d\r\n", card->total);
		printf("- Free sectors:  %d\r\n", card->free);
		printf("- Sector size:   %d bytes\r\n", FF_MAX_SS );
		break;
	 default:
		printf("- Unknown file system type %d\r\n", card->fs);
	}
	
	//printf("- Version: %d.%d\r\n", card->version[0], card->version[1]);
	//printf("- State: %02x\r\n", card->state);
	//rtc_time_t tme;
	//epoch_to_rtc_time(&tme, card->epoch);
	//printf("- Time: %02d/%02d/%02d %02d-%02d-%02d (%lu)\r\n",
	//	tme.year,tme.month,tme.day,tme.hour,tme.minute,tme.second, card->epoch);

}

uint8_t sd_card_state(uint8_t card_num, uint8_t state)
{
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
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
	if( card->write_addr > (card->end - nblocks) ) {
		return(1);
	}
	return(0);
}

int sd_card_set_number_of_blocks(uint8_t card_num, uint32_t nblocks)
{
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
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
	card->end = card->start + nblocks;

	// rewrite the header
//	sd_card_write_header(card);

	return(1);
}

uint32_t sd_card_get_number_of_blocks(uint8_t card_num)
{
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	return(card->end - card->start);
}

uint32_t sd_card_get_epoch(uint8_t card_num)
{
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(0);
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	return(card->epoch);
}

uint32_t sd_card_free_space(uint8_t card_num)
{
	sd_card_t *card = &sd_card_instance[card_num-1];
	return( card->end - card->write_addr);
}


//------------------------------------------------------------------------
// No file system (nfs) SD card utility functions
// There's no file system on the sd card, just a header and configuration block.

uint8_t sd_card_format(uint8_t card_num, char *name)
{
	uint8_t state = sd_card_init(card_num);
	if( !(state & SD_CARD_OK) ) {
		printf("sd_card_format: card init error\n\r");
		return( state );
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	// reset the card header block info
	card->start = SD_CARD_START_BLOCK;
	card->end = card->capacity * (1024 / SD_MMC_BLOCK_SIZE) - 1;
	// this will cause an overwrite of existing data
	card->write_addr = SD_CARD_START_BLOCK;
	card->read_addr = SD_CARD_START_BLOCK;
	
	// always overwrite card name
	strncpy(card->label, name, 8);

	// set header mod time
	rtc_get_epoch( &card->epoch );

	// rewrite the header
	sd_card_write_header(card);

	// note - the state is not written to the sd card header block
	card->state |= SD_CARD_FORMATED;
	
	card->fs = SD_FS_RAW;

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
	card->fs = SD_FS_RAW;
	
	return( card->state );
}


/*
 * sd_card_close - updates the card header and disables the card (powering off).
 * closing does not un-select the card. 
 */
void sd_card_close(uint8_t card_num)
{
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
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

static sd_mmc_err_t raw_write(uint8_t *buffer, uint16_t nblocks, uint32_t addr)
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

static sd_mmc_err_t raw_read(uint8_t *buffer, uint16_t nblocks, uint32_t addr)
{
	sd_mmc_err_t ret = 0;

	ret = sd_mmc_init_read_blocks(sd_card_slot, addr, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read: init_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}

	ret = sd_mmc_start_read_blocks(buffer, nblocks);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read: start_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	ret = sd_mmc_wait_end_of_read_blocks(sd_card_slot);
	if ( ret != SD_MMC_OK ) {
		printf("sd_card_read: wait_end_of_read_blocks FAILED, %d\n\r", ret);
		return(ret);
	}
	
	return(SD_MMC_OK);
}

uint16_t sd_card_write(uint8_t card_num, uint8_t *buffer, uint16_t nblocks)
{
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
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
	if( (addr < card->start) || (addr > (card->end - nblocks)) ) {
		printf("sd_card_write: address out or valid range\r\n");
		//printf("sd_card_write: card is full: %d %d %d\r\n", addr, card->end_block, nblocks);
		return(0);
	}

	// raw write to sd card
	if( raw_write(buffer, nblocks, addr) != SD_MMC_OK ) {
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
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
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
	if( (addr < card->start) || (addr > (card->end - nblocks)) ) {
		printf("sd_card_read: invalid address\r\n");
		return(0);
	}
	
	// raw write to sd card
	if( raw_read(buffer, nblocks, addr) != SD_MMC_OK ) {
		printf("sd_card_read: raw read error\r\n");
		return(0);
	}

	// update the current read address
	card->read_addr += nblocks;

	// update the card header
	sd_card_write_header(card);

	return( nblocks );
}


uint8_t sd_card_header_block[SD_MMC_BLOCK_SIZE];

int sd_card_read_header(sd_card_t *hdr)
{
	// current write address (in blocks)
	uint32_t addr = SD_CARD_HEADER_BLOCK;
	
	// raw read to sd card
	if( raw_read(sd_card_header_block, 1, addr) != SD_MMC_OK ) {
		printf("sd_card_read_header: raw read error\r\n");
		return(0);
	}

	int nrd = sd_card_parse_header(sd_card_header_block, hdr);
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
	int nwrt = sd_card_serialize_header(hdr, sd_card_header_block);
	
	// raw write to sd card
	if( raw_write(sd_card_header_block, 1, addr) != SD_MMC_OK) {
		printf("sd_card_write_header: raw write error\r\n");
		return(0);
	}
	return(nwrt);
}

uint8_t sd_card_config_block[SD_MMC_BLOCK_SIZE];

int sd_card_read_config(uint8_t card_num, wispr_config_t *hdr)
{
	if( card_num < 0 || card_num > NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(0);
	}
	//sd_card_t *card = &sd_card_instance[card_num-1];
	
	// current write address (in blocks)
	uint32_t addr = SD_CARD_CONFIG_BLOCK;
	
	// raw read to sd card
	if( raw_read(sd_card_config_block, 1, addr) != SD_MMC_OK ) {
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
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
		printf("Unknown SD number: enable SD Card Failed\n\r");
		return(0);
	}
	//sd_card_t *card = &sd_card_instance[card_num-1];
	
	// current write address (in blocks)
	uint32_t addr = SD_CARD_CONFIG_BLOCK;

	// unparse the config header into config block buffer
	int nwrt = wispr_serialize_config(hdr, sd_card_config_block);
	
	// raw write to sd card
	if( raw_write(sd_card_config_block, 1, addr) != SD_MMC_OK) {
		printf("sd_card_write_config: raw write error\r\n");
		return(0);
	}
	return(nwrt);
}

//
//
//
int sd_card_parse_header(uint8_t *buf, sd_card_t *hdr)
{
	if ( strncmp((const char *)buf, "WISPR", 5) != 0 ) {
		fprintf(stdout, "wispr_sd_card_parse_header: unrecognized header\r\n");
		return(0);
	}

	hdr->version[0] = buf[6];
	hdr->version[1] = buf[7];

	hdr->label[0] = buf[8];
	hdr->label[1] = buf[9];
	hdr->label[2] = buf[10];
	hdr->label[3] = buf[11];
	hdr->label[4] = buf[12];
	hdr->label[5] = buf[13];
	hdr->label[6] = buf[14];
	hdr->label[7] = buf[15];

	hdr->start  =  (uint32_t)buf[16];    // addr of first block (uint32_t)
	hdr->start |= ((uint32_t)buf[17] << 8);
	hdr->start |= ((uint32_t)buf[18] << 16);
	hdr->start |= ((uint32_t)buf[19] << 24);    // addr of first block (uint32_t)

	hdr->end  =  (uint32_t)buf[20];
	hdr->end |= ((uint32_t)buf[21] << 8);
	hdr->end |= ((uint32_t)buf[22] << 16);
	hdr->end |= ((uint32_t)buf[23] << 24);    // addr of last block (uint32_t)

	hdr->write_addr   = (uint32_t)buf[24];
	hdr->write_addr |= ((uint32_t)buf[25] << 8);
	hdr->write_addr |= ((uint32_t)buf[26] << 16);
	hdr->write_addr |= ((uint32_t)buf[27] << 24);    // addr of current write block (uint32_t)
	
	hdr->read_addr  =  (uint32_t)buf[28];
	hdr->read_addr |= ((uint32_t)buf[29] << 8);
	hdr->read_addr |= ((uint32_t)buf[30] << 16);
	hdr->read_addr |= ((uint32_t)buf[31] << 24);    // addr of current write block (uint32_t)
	
	hdr->epoch  =  (uint32_t)buf[32];
	hdr->epoch |= ((uint32_t)buf[33] << 8);
	hdr->epoch |= ((uint32_t)buf[34] << 16);
	hdr->epoch |= ((uint32_t)buf[35] << 24);    // addr of current write block (uint32_t)
	return(36);
}

int sd_card_serialize_header(sd_card_t *hdr, uint8_t *buf)
{
	buf[0]  = 'W';
	buf[1]  = 'I';
	buf[2]  = 'S';
	buf[3]  = 'P';
	buf[4]  = 'R';
	buf[5]  = '2';

	buf[6]  = (uint8_t)(hdr->version[0]);
	buf[7]  = (uint8_t)(hdr->version[1]);

	buf[8]  = hdr->label[0];
	buf[9]  = hdr->label[1];
	buf[10] = hdr->label[2];
	buf[11] = hdr->label[3];
	buf[12] = hdr->label[4];
	buf[13] = hdr->label[5];
	buf[14] = hdr->label[6];
	buf[15] = hdr->label[7];

	buf[16] = (uint8_t)(hdr->start >> 0);
	buf[17] = (uint8_t)(hdr->start >> 8);
	buf[18] = (uint8_t)(hdr->start >> 16);
	buf[19] = (uint8_t)(hdr->start >> 24);
	
	buf[20] = (uint8_t)(hdr->end >> 0);
	buf[21] = (uint8_t)(hdr->end >> 8);
	buf[22] = (uint8_t)(hdr->end >> 16);
	buf[23] = (uint8_t)(hdr->end >> 24);

	buf[24] = (uint8_t)(hdr->write_addr >> 0);
	buf[25] = (uint8_t)(hdr->write_addr >> 8);
	buf[26] = (uint8_t)(hdr->write_addr >> 16);
	buf[27] = (uint8_t)(hdr->write_addr >> 24);
	
	buf[28] = (uint8_t)(hdr->read_addr >> 0);
	buf[29] = (uint8_t)(hdr->read_addr >> 8);
	buf[30] = (uint8_t)(hdr->read_addr >> 16);
	buf[31] = (uint8_t)(hdr->read_addr >> 24);

	buf[32] = (uint8_t)(hdr->epoch >> 0);
	buf[33] = (uint8_t)(hdr->epoch >> 8);
	buf[34] = (uint8_t)(hdr->epoch >> 16);
	buf[35] = (uint8_t)(hdr->epoch >> 24);

	return(36);
}


//-------------------------------------------------------------------------
// FAT formated sd card utility functions
// 
// for fat function ref see: https://www.pjrc.com/tech/8051/ide/fat32.html
//
#include <ff.h>
FATFS fat_fs;
//FIL fat_file;


FRESULT sd_card_mount_fat(uint8_t card_num)
{
	uint8_t state = sd_card_init(card_num);
	if( !(state & SD_CARD_OK) ) {
		printf("sd_card_mount_fat: sd_card_init error %d\n\r", state);
		return( FR_NOT_READY );
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	memset(&fat_fs, 0, sizeof(FATFS));
	FRESULT res = f_mount(&fat_fs, "0", 1); // force mount
	if (res != FR_OK) {
		printf("sd_card_mount_fat: f_mount error %d\r\n", res);
		return(res);
	}

	// set header mod time
	rtc_get_epoch( &card->epoch );

	// Get volume information and free clusters
	DWORD free_clust;
	FATFS *fs = &fat_fs;
	res = f_getfree("0", &free_clust, &fs);
	if (res != FR_OK) {
		printf("sd_card_mount_fat: f_getfree error %d\r\n", res);
		return(res);
	}

	/* Get total sectors and free sectors */
	card->total = (fs->n_fatent - 2) * fs->csize;
	card->free = free_clust * fs->csize;

	// set label
	char label[12];
	sprintf(label, "WISPR_SD%d", card_num);
	f_setlabel(label);

	// always overwrite card name
	strcpy(card->label, label);

	// initialize cluster counts 
	card->write_addr = 0;
	card->read_addr = 0;
	card->start = 0;
	card->end = card->free;
	
	card->state |= SD_CARD_OPEN;
	card->fs = SD_FS_FAT;

	return(res);
}

//
// Formatting takes long so you need to disable the WDT to use it.
//
FRESULT sd_card_format_fat(uint8_t card_num)
{
	uint8_t state = sd_card_init(card_num);
	if( !(state & SD_CARD_OK) ) {
		printf("sd_card_mount_fat: card init error %d\n\r", state);
		return( FR_NOT_READY );
	}
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	printf("Formating disk - this could take a few minutes\n\r");

	// Format disk
	BYTE work[1024]; /* Work area (larger is better for processing time) */
	FRESULT res = f_mkfs("0", 0, work, 1024);
	if (res == FR_OK) {
		card->state |= SD_CARD_FORMATED;
	}

	card->fs = SD_FS_FAT;

	return(res);
}

FRESULT sd_card_umount_fat(uint8_t card_num)
{
	sd_card_t *card = &sd_card_instance[card_num-1];
	
	//printf("Unmounting FAT on SD Card.\r\n");

	FRESULT res = f_mount(0, "0", 0);
	if (res != FR_OK) {
		printf("sd_card_umount_fat: f_mount error %d\r\n", res);
	}
	
	if (res == FR_OK) {
		card->state &= ~SD_CARD_OPEN;
	}

	return(res);
}

FRESULT sd_card_open_fat(fat_file_t *ff, char *name, unsigned char mode, uint8_t card_num)
{	
	if( card_num < 1 || card_num > NUMBER_SD_CARDS ) {
		//printf("Unknown SD number: enable SD Card Failed\n\r");
		return(FR_INVALID_PARAMETER);
	}

	ff->state = 0;
	ff->card_num = card_num;
	ff->size = WISPR_MAX_FILE_SIZE;

	FRESULT res;
	res = f_open(&ff->file, name, mode); 
	if( res == FR_OK ) {
		ff->state = SD_FILE_OPEN;
		strncpy(ff->name, name, sizeof(ff->name));
	} else {
		printf("sd_card_f_open: f_open error %d with %s\r\n", res, name);	
	}
	
	return(res);
}

FRESULT sd_card_close_fat(fat_file_t *ff)
{
	if( !(ff->state & SD_FILE_OPEN) ) {
		return(FR_NOT_READY);
	}

	FRESULT res;
	res = f_close(&ff->file);
	if( res != FR_OK ) {
		printf("f_close FAILED: res %d\r\n", res);
	}

	ff->state == SD_FILE_CLOSED;
	
	return(res);
}

//
// Write a buffer of size nblocks to the open file.
// If the write fills the file (count >= size) the file full bit is set in the file state. 
// It's up the user to handle the file full state, such as closing the file and opening a new one
// because subsequent writes will just overfill the file. 
//
FRESULT sd_card_write_fat(fat_file_t *ff, uint8_t *buffer, uint16_t nblocks)
{
	if( !(ff->state & SD_FILE_OPEN) ) {
		printf("sd_card_write_fat: file not open\n\r");
		return(FR_NOT_READY);
	}
	
	// write to file on SD card
	size_t nwrt = 0;
	UINT nbytes = nblocks * SD_MMC_BLOCK_SIZE;
	FRESULT res = f_write(&ff->file, buffer, nbytes, &nwrt);
	if( nwrt != nbytes ) {
		printf("sd_card_write_fat: f_write error %d, %d\r\n", res, nwrt);
		return(res);
	}
	
	// update the current file block count
	ff->count += nblocks;

	// check if this write filled the file
	if( ff->count >= ff->size ) {
		ff->state |= SD_FILE_FULL;
	}
	
	// update the sd card total write count used to determine if card is full
	sd_card_instance[ff->card_num-1].write_addr += nblocks;

	return(res);
}


FRESULT sd_card_read_config_fat(char *filename, wispr_config_t *hdr)
{
	FIL file;
	FRESULT res;
	res = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
	if( res != FR_OK ) {
		if( res == FR_NO_FILE ) {
			printf("No configuration file found\r\n", res);
		} else {
			printf("sd_card_read_config_fat: f_open failed res %d\r\n", res);
		}
		return(res);
	}

	int len = 64;
	char buf[64];
	int v1, v2;
	wispr_config_t new;
	
	f_gets(buf, len, &file);
	sscanf(buf, "WISPR %d.%d configuration", &v1, &v2);
	new.version[1] = (uint8_t)v1;
	new.version[0] = (uint8_t)v2;

	char str[24];
	f_gets(buf, len, &file);
	sscanf(buf, "time: %s %s", str, &str[9]);
	str[8] = ' ';
	new.epoch = time_string_to_epoch(str);

	// scan file line by line
	while( 1 ) {		
		if( f_gets(buf, len, &file) == 0) break;
		//printf("%s\r\n", buf);
		sscanf(buf, "%s %d", str, &v1);
		if(strcmp(str, "mode:") == 0) new.mode = (uint8_t)v1;
		if(strcmp(str, "samples_per_block:") == 0) new.samples_per_block = (uint16_t)v1;
		if(strcmp(str, "sample_size:") == 0) new.sample_size = (uint8_t)v1;
		if(strcmp(str, "block_size:") == 0) new.block_size = (uint16_t)v1;
		if(strcmp(str, "sampling_rate:") == 0) new.sampling_rate = (uint32_t)v1;
		if(strcmp(str, "awake_time:") == 0) new.awake_time = (uint16_t)v1;
		if(strcmp(str, "sleep_time:") == 0) new.sleep_time = (uint16_t)v1;
		if(strcmp(str, "fft_size:") == 0) new.fft_size = (uint16_t)v1;
		if(strcmp(str, "fft_overlap:") == 0) new.fft_overlap = (uint16_t)v1;
		if(strcmp(str, "gain:") == 0) new.gain = (uint8_t)v1;
		if(strcmp(str, "adc_decimation:") == 0) new.adc_decimation = (uint8_t)v1;
		if(strcmp(str, "active_sd_card:") == 0) new.active_sd_card = (uint8_t)v1;
	}
	
	// validate new config
	// ...
	
	// copy new config into target
	memcpy(hdr, &new, sizeof(wispr_config_t));
	
	res = f_close(&file);
	if( res != FR_OK ) {
		printf("sd_card_read_config_fat: f_close failed res %d\r\n", res);
	}
	return(res);
}

FRESULT sd_card_write_config_fat(char *filename, wispr_config_t *hdr)
{	
	FRESULT res;
	FIL file;
	
	//res = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
	res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if( res != FR_OK ) {
		printf("sd_card_write_config_fat: failed to open %s, res %d\r\n", filename, res);
		return(res);
	}

	int nwrt = 0;
	//float buffer_duration =  (float)hdr->samples_per_block / (float)hdr->sampling_rate;

	nwrt += f_printf(&file, "WISPR %d.%d configuration\r\n", hdr->version[1], hdr->version[0]);
	nwrt += f_printf(&file, "time: %s\r\n", epoch_time_string(hdr->epoch));
	//nwrt += f_printf(&file, "epoch time: %lu\r\n", hdr->epoch);
	//nwrt += f_printf(&file, "state: %02x\r\n", hdr->state);
	nwrt += f_printf(&file, "mode: %02x\r\n", hdr->mode);
	nwrt += f_printf(&file, "samples_per_block: %d\r\n", hdr->samples_per_block);
	nwrt += f_printf(&file, "sample_size: %d\r\n", (int)hdr->sample_size);
	nwrt += f_printf(&file, "block_size: %d\r\n", (int)hdr->block_size);
	nwrt += f_printf(&file, "sampling_rate: %d\r\n", hdr->sampling_rate);
	nwrt += f_printf(&file, "gain: %d\r\n", hdr->gain);
	nwrt += f_printf(&file, "adc_decimation: %d\r\n", hdr->adc_decimation);
	nwrt += f_printf(&file, "awake_time: %d\r\n", hdr->awake_time);
	nwrt += f_printf(&file, "sleep_time: %d\r\n", hdr->sleep_time);
	nwrt += f_printf(&file, "fft_size: %d\r\n", hdr->fft_size);
	nwrt += f_printf(&file, "fft_overlap: %d\r\n", hdr->fft_overlap);
	nwrt += f_printf(&file, "active_sd_card: %d\r\n", hdr->active_sd_card);

	res = f_close(&file);
	if( res != FR_OK ) {
		printf("sd_card_read_config_fat: f_close failed res %d\r\n", res);
	}
	return(nwrt);
}

