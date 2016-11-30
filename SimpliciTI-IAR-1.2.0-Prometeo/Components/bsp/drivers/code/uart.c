/**************************************************************************************************
  Filename:       uart.c
  Revised:        $Date: 2009-08-17 10:50:58 -0700 (Mon, 17 Aug 2009) $
  Author:         $Author: jnoxon $

  Description:    This file supports the SimpliciTI-compatible UART driver.

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
#include "bsp.h"
#include "uart.h"
#include "uart_intfc.h"
#include "PGN_protocol.h"
#include "timer_b.h"
#include "msp430.h"

// Receive buffer for the UART.  Incoming bytes need a place to go immediately,
// otherwise there might be an overrun when the next comes in.  The USCI ISR
// puts them here.
uint8_t  bcUartRcvBuf[BC_RXBUF_SIZE];

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */
uint16_t bcUartRcvBufIndex = 0;
uint8_t  bcUartRxThreshReached = 0;



uint8_t  rxUart = FALSE;
uint8_t  uartTimeoutExpired = FALSE;
uint16_t counterTimer;

/******************************************************************************
 * LOCAL FUNCTIONS
 */

void uart_rx_irq( void );

/******************************************************************************
 * @fn          uart_rx_irq
 *
 * @brief       RX interrupt service routine
 *
 * input parameters
 *
 * output parameters
 *
 * @return   
 */
//void uart_rx_irq( void )
//  {
  /* read in the received data, this will clear the interrupt also */
   // uart_rxData(UCA0RXBUF);
//return;
//  }



/******************************************************************************
 * GLOBAL FUNCTIONS
 */ 

/******************************************************************************
 * @fn          uart_init
 *
 * @brief       Configures UART and sets up transmit and receive interrupts
 *
 * input parameters
 *
 * output parameters
 *
 * @return   
 */

//igual que bcUartInitPGN(void) (USCI_A0, 9600 bps)
//void uart_init( void )
//  {
//  
//  P3SEL = BIT3+BIT4;                        // P3.3 and P3.4 = USCI_A1 TXD/RXD
//  //w/ ACLK enabled: USCI A0
//  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
//  UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
//  UCA0BR0 = 0x03;                           // 32kHz/9600=3.41 (see User's Guide)
//  UCA0BR1 = 0x00;                           //
//  UCA0MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0
//  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
//
//  /* enable receive interrupts, they are always welcome. */
//    UCA0IE |= UCRXIE;
//
//    return;
//  }

//igual que bcUartInitPGN(void) (USCI_A0, 9600 bps)
void uart_init( void )
  {
  
    UCA0CTL1 |= UCSWRST;        // Put the USCI state machine in reset
    UCA0CTL1 |= UCSSEL_1;       // CLK = ACLK

    // Set the baudrate
    UCA0BR0 = 0x03;                           // 32kHz/9600=3.41 (see User's Guide)
    UCA0BR1 = 0x00;                           //
    UCA0MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0

    P3SEL |= BIT3+BIT4;         // Configure these pins as TXD/RXD

    UCA0CTL1 &= ~UCSWRST;       // Take the USCI out of reset
    UCA0IE |= UCRXIE;           // Enable the RX interrupt.  Now, when bytes are
                                // rcv'ed, the USCI_A1 vector will be generated.
  }

//Sends 'len' bytes, starting at 'buf'
void bcUartSend(uint8_t * buf, uint8_t len)
{
    uint8_t i = 0;

    // Write each byte in buf to USCI TX buffer, which sends it out
    while (i < len)
    {
        UCA0TXBUF = *(buf+(i++));

        // Wait until each bit has been clocked out...
        while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
    }
}

// Copies into 'buf' whatever bytes have been received on the UART since the
// last fetch.  Returns the number of bytes copied.
uint16_t bcUartReceiveBytesInBuffer(uint8_t* buf)
{
    uint16_t i, count;

    // Hold off ints for incoming data during the copy
    UCA1IE &= ~UCRXIE;

    for(i=0; i<bcUartRcvBufIndex; i++)
    {
        buf[i] = bcUartRcvBuf[i];
    }

    count = bcUartRcvBufIndex;
    bcUartRcvBufIndex = 0;     // Move index back to the beginning of the buffer
    bcUartRxThreshReached = 0;

    // Restore USCI interrupts, to resume receiving data.
    UCA1IE |= UCRXIE;

    return count;
}

// Reads a command from the UART (keeps reading until EOF) and stores it in buf
uint16_t bcUartReadCommand(uint8_t* buf)
{
    uint16_t i, count;
    uint16_t charPair = 0x0000;

    while (charPair != EOF)
    {
      // Hold off ints for incoming data during the copy
      UCA0IE &= ~UCRXIE;
      
      for(i=0; i<bcUartRcvBufIndex; i++)
      {
          buf[i] = bcUartRcvBuf[i];
      }
  
      count = bcUartRcvBufIndex;
      charPair = (buf[bcUartRcvBufIndex-2] << 8) | buf[bcUartRcvBufIndex-1];  // Last two bytes
  
      // Restore USCI interrupts, to resume receiving data.
      UCA0IE |= UCRXIE;
    }
    
    bcUartRcvBufIndex = 0;     // Move index back to the beginning of the buffer
    bcUartRxThreshReached = 0;
    return count;
}




// Reads a command from the UART (keeps reading until EOF or
// the timeout expires) and stores it in buf
uint16_t bcUartReadCommandTimeout(uint8_t* buf)
{
   uint16_t i, count;
   uint16_t charPair = 0x0000;
    uartTimeoutExpired = FALSE;

    while ((charPair != EOF)&&(!uartTimeoutExpired))
    {
      rxUart = FALSE;
      uartTimeoutExpired = FALSE;
      
      // Start the timer
      counterTimer = 0;
      timerInit(50000);
           
      // Wait for uart or timeout
      while ((!rxUart)&&(!uartTimeoutExpired));
      
      if(!uartTimeoutExpired) {
        // Hold off ints for incoming data during the copy
        UCA0IE &= ~UCRXIE;
        for(i=0; i<bcUartRcvBufIndex; i++)
        {
            buf[i] = bcUartRcvBuf[i];
        }
        count = bcUartRcvBufIndex;
        charPair = (buf[bcUartRcvBufIndex-2] << 8) | buf[bcUartRcvBufIndex-1];  // Last two bytes
      
        // Restore USCI interrupts, to resume receiving data.
        UCA0IE |= UCRXIE;
      }  
      
      else
      {
        count = -1;
      }
    }
    
    bcUartRcvBufIndex = 0;     // Move index back to the beginning of the buffer
    bcUartRxThreshReached = 0;
    return count;
}

void timerInit(uint16_t rate)
{
  TIMER_B_startUpMode(   TIMER_B0_BASE,
                            TIMER_B_CLOCKSOURCE_SMCLK,
                            TIMER_B_CLOCKSOURCE_DIVIDER_1,
                            rate,
                            TIMER_B_TBIE_INTERRUPT_DISABLE,
                            TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE,
                            TIMER_B_DO_CLEAR
                            );
}

// The USCI_A1 receive interrupt service routine (ISR).  Executes every time a
// byte is received on the back-channel UART.
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void bcUartISR(void);
#pragma vector=USCI_A0_VECTOR
__interrupt void bcUartISR(void)
{
    bcUartRcvBuf[bcUartRcvBufIndex++] = UCA0RXBUF;  // Fetch the byte, store
                                                    // it in the buffer.
    rxUart = TRUE;

    // Wake main, to fetch data from the buffer.
    if(bcUartRcvBufIndex >= BC_RX_WAKE_THRESH)
    {
        bcUartRxThreshReached = 1;
        __bic_SR_register_on_exit(LPM3_bits);       // Exit LPM0-3
    }
}

 //This is the Timer B0 interrupt vector service routine.
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR(void);
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR(void)
{
   counterTimer++;
   if (counterTimer == TIMEOUT_UART) {
      uartTimeoutExpired = TRUE;
      counterTimer = 0;
   }
}

//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//  #pragma vector=USCI_A0_VECTOR
//__interrupt void uart_enter_irq( void );
//
//  #pragma vector=USCI_A0_VECTOR
//__interrupt void uart_enter_irq( void )
//#else
//#error Compiler not supported!
//#endif
//{
//  switch(__even_in_range(UCA0IV,4))
//  {
//  case 0:break;                             // Vector 0 - no interrupt
//  case 2:                                   // Vector 2 - RXIFG
//  uart_rx_irq( );
//   if(bcUartRcvBufIndex >= BC_RX_WAKE_THRESH)
//    {
//        bcUartRxThreshReached = 1;
//        __bic_SR_register_on_exit(LPM3_bits);       // Exit LPM0-3
//    }
//    break;
//  case 4:break;                             // Vector 4 - TXIFG
//   //do nothing
//  default: break;
//  }
//}

//void uart_rx_irq( void )
//  {
//  /* read in the received data, this will clear the interrupt also */
//    uart_rxData(UCA0RXBUF);
//    //bcUartRcvBuf[bcUartRcvBufIndex++] = UCA0RXBUF;
//    //bcUartReadCommandTimeout(UCA0RXBUF);
//return;
//  }

