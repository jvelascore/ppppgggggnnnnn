/**************************************************************************************************
  Filename:       uart.h
  Revised:        $Date: 2009-08-17 10:50:58 -0700 (Mon, 17 Aug 2009) $
  Author:         $Author: jnoxon $

  Description:    This header file supports the SimpliciTI-compatible UART driver.

  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef uart_h
#define uart_h

#define UCA1_OS   1    // 1 = oversampling mode, 0 = low-freq mode
#define UCA1_BR0  17   // Value of UCA1BR0 register
#define UCA1_BR1  0    // Value of UCA1BR1 register
#define UCA1_BRS  0    // Value of UCBRS field in UCA1MCTL register
#define UCA1_BRF  6    // Value of UCBRF field in UCA1MCTL register


/******************************************************************************
 * INCLUDES
 */
#include <stdbool.h> /* supports bool, true, and false */

#include <stddef.h>  /* supports NULL macro */

#include "bsp.h"

#include "c/stdint.h"


/*The size of the UART receive buffer.  Set smaller if RAM is in short supply.
Set larger if larger data chunks are to be received, or if the application
can't process incoming data very often   */
#define BC_RXBUF_SIZE  (128)

/* The threshold within bcUartRcvBuf at which main() will be awakened.  Must
be less than BC_RXBUF_SIZE.  A value of '1' will alert main() whenever
even a single byte is received.  If no wake is desired, set to
BC_RXBUF_SIZE+1     */
#define BC_RX_WAKE_THRESH  (1)

/* ------------------------------------------------------------------------------------------------
 *                                Function Prototypes
 * ------------------------------------------------------------------------------------------------
 */




/* initializes the uart for operation.  this function should be called before
 * any other uart functions are called. */
void uart_init( void );
void bcUartSend(uint8_t* buf, uint8_t len);
uint16_t bcUartReceiveBytesInBuffer(uint8_t* buf);
uint16_t bcUartReadCommand(uint8_t* buf);
uint16_t bcUartReadCommandTimeout(uint8_t* buf);
void timerInit(uint16_t rate);


#endif /* uart_h */