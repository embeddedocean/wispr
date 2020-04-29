/**
 * \file
 *
 * \brief Implementation of low level disk I/O module skeleton for FatFS.
 *
 * Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

#include "compiler.h"
#include "diskio.h"
#include "ctrl_access.h"
#include "sd_mmc.h"

//#include "sd_mmc_mem.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
# include <rtc.h>


/**
 * \defgroup thirdparty_fatfs_port_group Port of low level driver for FatFS
 *
 * Low level driver for FatFS. The driver is based on the ctrl access module
 * of the specific MCU device.
 *
 * @{
 */

/** Default sector size */
#define SECTOR_SIZE_DEFAULT 512

/** Supported sector size. These values are based on the LUN function:
 * mem_sector_size(). */
#define SECTOR_SIZE_512  1
#define SECTOR_SIZE_1024 2
#define SECTOR_SIZE_2048 4
#define SECTOR_SIZE_4096 8

static uint32_t sd_card_last_sector_num;

/**
 * \brief Initialize a disk.
 *
 * \param drv Physical drive number (0..).
 *
 * \return 0 or disk status in combination of DSTATUS bits
 *         (STA_NOINIT, STA_PROTECT).
 */
DSTATUS disk_initialize(BYTE drv)
{
	int i;
	Ctrl_status mem_status;

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Check LUN ready (USB disk report CTRL_BUSY then CTRL_GOOD) */
	for (i = 0; i < 2; i ++) {
		//mem_status = mem_test_unit_ready(drv);
		mem_status = sd_mmc_test_unit_ready(0);
		if (CTRL_BUSY != mem_status) {
			break;
		}
	}
	if (mem_status != CTRL_GOOD) {
		return STA_NOINIT;
	}

	/* Check Write Protection Status */
	//if (mem_wr_protect(drv)) {
	//	return STA_PROTECT;
	//}

	// cj - speed up
	sd_mmc_read_capacity(drv, &sd_card_last_sector_num);

	/* The memory should already be initialized */
	return 0;
}

/**
 * \brief  Return disk status.
 *
 * \param drv Physical drive number (0..).
 *
 * \return 0 or disk status in combination of DSTATUS bits
 *         (STA_NOINIT, STA_NODISK, STA_PROTECT).
 */
DSTATUS disk_status(BYTE drv)
{
	//switch (mem_test_unit_ready(drv)) {
	switch (sd_mmc_test_unit_ready(0)) {
	case CTRL_GOOD:
		return 0;
	case CTRL_NO_PRESENT:
		return STA_NOINIT | STA_NODISK;
	default:
		return STA_NOINIT;
	}
}

/**
 * \brief  Read sector(s).
 *
 * \param drv Physical drive number (0..).
 * \param buff Data buffer to store read data.
 * \param sector Sector address (LBA).
 * \param count Number of sectors to read (1..255).
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
DRESULT disk_read (BYTE drv, BYTE* buff, LBA_t sector, UINT count)
{
//	uint8_t uc_sector_size = mem_sector_size(drv);
	uint8_t uc_sector_size = 1;
	uint32_t i;
	uint32_t ul_last_sector_num;

	if (uc_sector_size == 0) {
		return RES_ERROR;
	}

	/* Check valid address */
	//mem_read_capacity(drv, &ul_last_sector_num);
	sd_mmc_read_capacity(drv, &ul_last_sector_num);
	if ((sector + count * uc_sector_size) >
			(ul_last_sector_num + 1) * uc_sector_size) {
		return RES_PARERR;
	}

	/* Read the data */
	for (i = 0; i < count; i++) {
		//if (memory_2_ram(drv, sector + uc_sector_size * i, buff + uc_sector_size * SECTOR_SIZE_DEFAULT * i) != CTRL_GOOD) {
		if (sd_mmc_mem_2_ram(drv, sector + uc_sector_size * i, buff + uc_sector_size * SECTOR_SIZE_DEFAULT * i) != CTRL_GOOD) {
			return RES_ERROR;
		}
	}

	return RES_OK;

}

/**
 * \brief  Write sector(s).
 *
 * The FatFs module will issue multiple sector transfer request (count > 1) to
 * the disk I/O layer. The disk function should process the multiple sector
 * transfer properly. Do not translate it into multiple sector transfers to the
 * media, or the data read/write performance may be drastically decreased.
 *
 * \param drv Physical drive number (0..).
 * \param buff Data buffer to store read data.
 * \param sector Sector address (LBA).
 * \param count Number of sectors to read (1..255).
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
DRESULT disk_write (BYTE drv, const BYTE* buff, LBA_t sector, UINT count)
{
	//uint8_t uc_sector_size = mem_sector_size(drv);
	uint8_t uc_sector_size = 1;
	uint32_t ul_last_sector_num;

	if (uc_sector_size == 0) {
		return RES_ERROR;
	}

	/* Check valid address */
	//mem_read_capacity(drv, &ul_last_sector_num);
	//sd_mmc_read_capacity(drv, &ul_last_sector_num);
	//if ((sector + count * uc_sector_size) >
	//		(ul_last_sector_num + 1) * uc_sector_size) {
	//	return RES_PARERR;
	//}
	//cj speed up
	if ((sector + count * uc_sector_size) > (sd_card_last_sector_num + 1) * uc_sector_size) {
		return RES_PARERR;
	}

	/* Write the data */
	/*
	uint32_t i;
	for (i = 0; i < count; i++) {
		//if (ram_2_memory(drv, sector + uc_sector_size * i,
		if (sd_mmc_ram_2_mem(0, sector + uc_sector_size * i,
				buff + uc_sector_size * SECTOR_SIZE_DEFAULT * i) !=
				CTRL_GOOD) {
			return RES_ERROR;
		}
	}
	*/
	//cj this should be faster
	if (SD_MMC_OK != sd_mmc_init_write_blocks(0, sector, count)) {
		return RES_ERROR;
	}
	if (SD_MMC_OK != sd_mmc_start_write_blocks(buff, count)) {
		return RES_ERROR;
	}
	if (SD_MMC_OK != sd_mmc_wait_end_of_write_blocks(false)) {
		return RES_ERROR;
	}
	
	
	return RES_OK;

}


/**
 * \brief  Miscellaneous functions, which support the following commands:
 *
 * CTRL_SYNC    Make sure that the disk drive has finished pending write
 * process. When the disk I/O module has a write back cache, flush the
 * dirty sector immediately.
 * In read-only configuration, this command is not needed.
 *
 * GET_SECTOR_COUNT    Return total sectors on the drive into the DWORD variable
 * pointed by buffer.
 * This command is used only in f_mkfs function.
 *
 * GET_BLOCK_SIZE    Return erase block size of the memory array in unit
 * of sector into the DWORD variable pointed by Buffer.
 * When the erase block size is unknown or magnetic disk device, return 1.
 * This command is used only in f_mkfs function.
 *
 * GET_SECTOR_SIZE    Return sector size of the memory array.
 *
 * \param drv Physical drive number (0..).
 * \param ctrl Control code.
 * \param buff Buffer to send/receive control data.
 *
 * \return RES_OK for success, otherwise DRESULT error code.
 */
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
	DRESULT res = RES_PARERR;

	switch (ctrl) {
	case GET_BLOCK_SIZE:
		*(DWORD *)buff = 1;
		res = RES_OK;
		break;

	/* Get the number of sectors on the disk (DWORD) */
	case GET_SECTOR_COUNT:
	{
		uint32_t ul_last_sector_num;

		/* Check valid address */
		//mem_read_capacity(drv, &ul_last_sector_num);
		sd_mmc_read_capacity(0, &ul_last_sector_num);

		*(DWORD *)buff = ul_last_sector_num + 1;

		res = RES_OK;
	}
	break;

	/* Get sectors on the disk (WORD) */
	case GET_SECTOR_SIZE:
	{
		//uint8_t uc_sector_size = mem_sector_size(drv);
		uint8_t uc_sector_size = 1;

		if ((uc_sector_size != SECTOR_SIZE_512) &&
				(uc_sector_size != SECTOR_SIZE_1024) &&
				(uc_sector_size != SECTOR_SIZE_2048) &&
				(uc_sector_size != SECTOR_SIZE_4096)) {
			/* The sector size is not supported by the FatFS */
			return RES_ERROR;
		}

		*(U8 *)buff = uc_sector_size * SECTOR_SIZE_DEFAULT;

		res = RES_OK;
	}
	break;

	/* Make sure that data has been written */
	case CTRL_SYNC:
		//if (mem_test_unit_ready(drv) == CTRL_GOOD) {
		if (sd_mmc_test_unit_ready(0) == CTRL_GOOD) {
			res = RES_OK;
		} else {
			res = RES_NOTRDY;
		}
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
