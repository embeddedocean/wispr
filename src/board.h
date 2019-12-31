/**
 * \file
 *
 * \brief Standard board header file.
 *
 * This file includes the appropriate board header file according to the
 * defined board (parameter BOARD).
 *
 * Copyright (c) 2009-2018 Microchip Technology Inc. and its subsidiaries.
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
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef _BOARD_H_
#define _BOARD_H_

/**
 * \defgroup group_common_boards Generic board support
 *
 * The generic board support module includes board-specific definitions
 * and function prototypes, such as the board initialization function.
 *
 * \{
 */

#include "compiler.h"

#ifdef __cplusplus
extern "C" {
#endif


/*! Base Boards */
#define WISPR_V2_0                     1  //!< WISPR V2.0 board.
#define WISPR_V2_1                     2  //!< WISPR V2.1 board.

/* Extension Boards */
#define ADC_LTC2512_V1_0                      1  //!< LTC2512 ADC board

/*
* The symbol BOARD can be defined here or passed as a toolchain compiler symbol 
*/
#define BOARD		WISPR_V2_0
#define EXT_BOARD	ADC_LTC2512_V1_0


#if BOARD == WISPR_V2_0
#  include "board_v2.0.h"
#elif BOARD == WISPR_V2_1
#  include "board_v2.1.h"
#else
#  error No known board defined
#endif

#if (defined EXT_BOARD)
#  if EXT_BOARD == ADC_LTC2512_V1_0
#    include "ltc2512.h"
#  elif EXT_BOARD == USER_EXT_BOARD
    // User-reserved area: #include the header file of your extension board here
    // (if any).
#  endif
#endif

#ifdef __cplusplus
}
#endif


#endif  // _BOARD_H_
