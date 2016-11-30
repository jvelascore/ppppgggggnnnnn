/**************************************************************************************************
  Filename:       uart_intfc.c
  Revised:        $Date: 2009-08-17 10:50:58 -0700 (Mon, 17 Aug 2009) $
  Author:         $Author: jnoxon $

  Description:    This file supports the SimpliciTI-compatible UART API functions.

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

/******************************************************************************
 * INCLUDES
 */
#include "uart_intfc.h"
#include <stdlib.h>

#include "bsp.h"
#ifdef FREQUENCY_HOPPING
#include "nwk_pll.h"
#endif

/******************************************************************************
 * CONSTANTS AND DEFINES
 */
#ifndef RX_TX_BUFFER_SIZE       /* this value must be at least 2. */
#define RX_TX_BUFFER_SIZE 50
#endif

/******************************************************************************
 * MACROS
 */
#define RX_TX_BUFFER_THROTTLE_LIMIT ( ( ( UART_BAUD_RATE ) > 15000 \
                                       && ( RX_TX_BUFFER_SIZE ) > 8 ) \
                                             ? ( ( RX_TX_BUFFER_SIZE ) - 4 )\
                                             : ( ( RX_TX_BUFFER_SIZE ) - 1 ) )

/******************************************************************************
 * LOCAL VARIABLES
 */
static unsigned int f_readyToTransmit;
static uint8_t rx_buff[RX_TX_BUFFER_SIZE];
static uint8_t tx_buff[RX_TX_BUFFER_SIZE];
static uint8_t* rx_head = rx_buff;
static uint8_t* rx_tail = rx_buff;
static uint8_t* tx_head = tx_buff;
static uint8_t* tx_tail = tx_buff;

/******************************************************************************
 * GLOBAL SHARED VARIABLES
 */
uint8_t f_uartReceivedMessage;

/******************************************************************************
 * LOCAL FUNCTIONS
 */



/* uart data handler function prototypes */
bool tx_handler( uint8_t* c );
bool rx_handler( uint8_t c );

int buffer_free_space( uint8_t* head, uint8_t* tail );
int buffer_used_space( uint8_t* head, uint8_t* tail );
bool push_buffer( uint8_t** head, uint8_t* tail,
                  uint8_t* buff, uint8_t* data, int len );
int pop_buffer( uint8_t* head, uint8_t** tail,
                uint8_t* buff, uint8_t* data, int max_len );
int flush_buffer(uint8_t** head, uint8_t** tail, uint8_t* buff);

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          uart_intfc_init
 *
 * @brief       Initialize UART interface. Buffer pointers are initialized.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */
void uart_intfc_init( void )
  {
  /* initialize the buffer pointers in case we are re-initialized */
  rx_head = rx_buff;
  rx_tail = rx_buff;
  tx_head = tx_buff;
  tx_tail = tx_buff;

  /* put ready to trasmit flag = 0, so we cannot transmit yet. */
  f_readyToTransmit = 0;  // put ready to transmit flag = 0
  f_uartReceivedMessage = 0;
  //f_uartHeaderByte = 1;
  //uartPayloadData = 0;
  /* initialize the uart for operations */
  uart_init( );
 
  return;
  }

/******************************************************************************
 * @fn          buffer_free_space
 *
 * @brief       Calculates amount of freespace in buffer from <head> and <tail>
 *              addresses
 *
 * input parameters
 * @param   head       - pointer to buffer head
 * @param   tail       - pointer to buffer tail
 *
 * output parameters
 *
 * @return   Buffer free space count in bytes
 */
int buffer_free_space( uint8_t* head, uint8_t* tail )
  {
  /* the free count is the buffer size minus the used count minus one
   * because we don't want the pointers to ever get back on top of each other
   * because that would indicate an empty buffer. */
  return RX_TX_BUFFER_SIZE - buffer_used_space( head, tail ) - 1;
  }

/******************************************************************************
 * @fn          buffer_used_space
 *
 * @brief       Calculates amount of used space in buffer from <head> and <tail>
 *              addresses
 *
 * input parameters
 * @param   head       - pointer to buffer head
 * @param   tail       - pointer to buffer tail
 *
 * output parameters
 *
 * @return   Buffer used space count in bytes
 */
int buffer_used_space( uint8_t* head, uint8_t* tail )
  {
  ptrdiff_t used;
  
  used = head - tail; /* get used count */
  if( used < 0 ) /* if the pointers were wrapped */
    used += RX_TX_BUFFER_SIZE; /* correct the count */
  
  return used; /* return used count */
  }

/******************************************************************************
 * @fn          push_buffer
 *
 * @brief       Pushes bytes of data onto the specified buffer. Assumes on 
 *              entry that <data>, <buff>, <tail>, and <head> are all valid 
 *              pointers
 *
 * input parameters
 * @param   tail       - pointer to buffer tail
 * @param   buff       - pointer to buffer (push target)
 * @param   data       - pointer to data to be pushed into buffer
 * @param   len        - length in bytes of data to be pushed
 *
 * output parameters
 * @param   head       - updated buffer head pointer
 *
 * @return   status of operation.
 *             true          Data successfully pushed into buffer
 *             false         Data not pushed into buffer
 */

bool push_buffer( uint8_t** head, uint8_t* tail,
                  uint8_t* buff, uint8_t* data, int len )
  {
  uint8_t* local_head;
  uint8_t* local_tail = tail;

  BSP_CRITICAL_STATEMENT( local_head = *head );
  
  /* if no room in the buffer */
  if( buffer_free_space( local_head, local_tail ) < len )
    return false; /* indicate failure to enqueue message */
  
  /* there is room for the data, put in the buffer */
  
  do /* put the data in the buffer */
    {
    if( local_head == buff + RX_TX_BUFFER_SIZE ) /* if wrapping around */
      local_head = buff; /* reset pointer */
    
    *local_head++ = *data++; /* copy over this byte of data */
    } while( --len > 0 ); /* copy all the data to the buffer */
  
  BSP_CRITICAL_STATEMENT( *head = local_head ); /* update reference value */
  
  return true;
  }

/******************************************************************************
 * @fn          pop_buffer
 *
 * @brief       Pops the specified number of bytes off of the specified buffer. 
 *              Assumes on entry that <data>, <buff>, <tail>, and <head> are 
 *              all valid pointers.
 *
 * input parameters
 * @param   head       - pointer to buffer head
 * @param   buff       - pointer to buffer (pop source)
 * @param   data       - pointer to location to store popped data
 * @param   len        - amount of bytes to be popped from buffer
 *
 * output parameters
 * @param   tail       - updated buffer tail pointer
 *
 * @return   number of bytes popped from buffer
 */

int pop_buffer( uint8_t* head, uint8_t** tail,
                uint8_t* buff, uint8_t* data, int max_len )
  {
  uint8_t* local_tail;
  uint8_t* local_head = head;
  int cnt = 0;

  BSP_CRITICAL_STATEMENT( local_tail = *tail );
  
  /* if the buffer is empty or no data requested */
  if( local_tail == local_head || max_len <= 0 )
    return 0; /* indicate so */
  
  do /* retrieve the data from the buffer */
    {
    if( local_tail == buff + RX_TX_BUFFER_SIZE ) /* if wrapping around */
      local_tail = buff; /* reset pointer */
    
    *data++ = *local_tail++; /* copy data from buffer */
      
    /* while the user needs more data and there is data left in the fifo */
    } while( ++cnt < max_len && local_tail != local_head );
    
  BSP_CRITICAL_STATEMENT( *tail = local_tail ); /* update reference value */
  
  return cnt; /* return number of characters retrieved from the buffer */
  }

/******************************************************************************
 * @fn          tx_peek
 *
 * @brief       Returns the number of bytes of free space in the transmit FIFO. 
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Number of bytes of free space in the transmit FIFO
 */

int tx_peek( void )
  {
  uint8_t* head;
  uint8_t* tail;
  
  BSP_CRITICAL_STATEMENT( head = tx_head; tail = tx_tail );
  
  return buffer_free_space( head, tail );
  }

/******************************************************************************
 * @fn          rx_peek
 *
 * @brief       Returns the number of bytes currently available to be read from
 *              the receive FIFO. 
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Number of bytes of available in the receive FIFO
 */

int rx_peek( void )
  {
  uint8_t* head;
  uint8_t* tail;
  
  BSP_CRITICAL_STATEMENT( head = rx_head; tail = rx_tail );
  
  return buffer_used_space( head, tail );
  }

/* Empty (reset) the buffer */

int flush_buffer(uint8_t** head, uint8_t** tail, uint8_t* buff){
  BSP_CRITICAL_STATEMENT( *head = buff; *tail = buff);
  return 0;
}



/*
Retrieves data from TX buffer and sends it
*/              
              
int uart_txData(uint8_t *data, uint8_t arrayLength){

  bool status = 1;
  uint8_t* head;
  uint8_t c; // data to send

  /*First of all: send data to buffer and wait until it is done*/
  uart_txToBuffer(data, arrayLength);
  // wait until TX is ready:
  while (f_readyToTransmit == 0);
  // When we can transmit...
  /* get current state of head pointer */
  BSP_CRITICAL_STATEMENT( head = tx_head );
  /* get data off of the transmit buffer */
  while(arrayLength--){
  pop_buffer( head, &tx_tail, tx_buff, &c, 1 );
  while (!(UCA0IFG&UCTXIFG)); // USCI_A0 TX buffer ready?
  UCA0TXBUF = *(data);
  data++;
  //BSP_Delay(50);
  }
  /* We have transmitted, so put flag to zero until we push another message into buffer...*/
  f_readyToTransmit = 0;
  /* check status of buffer */
  BSP_CRITICAL_STATEMENT( status = tx_head != tx_tail );
  return status; /* indicate if this is the last byte in the buffer */
}

/*
Gets data from line (UART) and puts it in RX buffer
*/ 

int uart_rxData(uint8_t c){

  uint8_t* tail;

  /* get current state of tail pointer */
  BSP_CRITICAL_STATEMENT( tail = rx_tail );

  
  /* put data onto the receive buffer */
  /*  convert data from ASCII to non ASCII -> hexadecimal  */
    
    push_buffer( &rx_head, tail, rx_buff, &c, 1 );
    uart_notifyReceivedMessage(); // notify main thread
 
  return true; /* always accept data received from the uart */

}

/*
Pushes data into TX buffer
*/

int uart_txToBuffer(uint8_t *data, uint8_t len){

  bool status;
  uint8_t* tail;
  
  /* get current state of tail pointer */
  BSP_CRITICAL_STATEMENT( tail = tx_tail );

  /* put data into transmit buffer */
  status = push_buffer( &tx_head, tail, tx_buff, (uint8_t*)data, len );

  if( status != false ) /* if data was put in the buffer properly */
    f_readyToTransmit = 1;

  return status; /* return status */
}

/*
Pulls data from RX buffer
*/

int uart_rxFromBuffer(uint8_t *data, uint8_t max_len){
  int cnt;
  uint8_t* head;
  
  /* get current state of head pointer */
  BSP_CRITICAL_STATEMENT( head = rx_head );
  
  /* retrieve data from buffer */
  cnt = pop_buffer( head, &rx_tail, rx_buff, data, max_len );

  return cnt; /* indicate the number of bytes retrieved from the buffer */
}
        
void uart_notifyReceivedMessage(void){
  f_uartReceivedMessage++;
  //uartPayloadData = 0;
  //f_uartHeaderByte = 1;  // next byte will be from a next message
}     