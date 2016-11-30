/*----------------------------------------------------------------------------
 *  Demo Application for SimpliciTI
 *
 *  L. Friedman
 *  Texas Instruments, Inc.
 *---------------------------------------------------------------------------- */

/**********************************************************************************************
  Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.

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

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_pll.h"

#include "nwk.h"

#include "nwk_frame.h"

#include <string.h>

#ifdef MRFI_CC430                // Esto es añadido 
  #include "uart_intfc_cc430.h"  // Si MRFI_CC430 esta definido,
#else                            // incluye "uart_intfc_cc430.h",
  #include "uart_intfc.h"        // si no, "uart_intfc.h"
#endif                           // 

#ifndef APP_AUTO_ACK
#error ERROR: Must define the macro APP_AUTO_ACK for this application.
#endif

/* variables y funciones*/

void toggleLED(uint8_t);

void UART_init(void);
void sw_init(void); // initialize variables
void uart_processMessage(void);
void pgn_processMessage(uint8_t*, uint8_t);
void pgn_requestDescription(void);
void pgn_createNetwork(void);
void rf_sendMessage(uint8_t* data, uint8_t id,  uint8_t len);

static uint8_t dataPayload[MAX_APP_PAYLOAD];
static uint8_t* p_dataPayload;
static uint8_t f_isUartHeaderByte;
static bool f_createNetwork;


static void linkTo(void);

static uint8_t  sTid = 0;
static linkID_t sLinkID1 = 0;

#define SPIN_ABOUT_A_SECOND   NWK_DELAY(1000)
#define SPIN_ABOUT_A_QUARTER_SECOND   NWK_DELAY(250)

/* How many times to try a Tx and miss an acknowledge before doing a scan */
#define MISSES_IN_A_ROW  2

void main (void)
{
  BSP_Init();

  /* If an on-the-fly device address is generated it must be done before the
   * call to SMPL_Init(). If the address is set here the ROM value will not
   * be used. If SMPL_Init() runs before this IOCTL is used the IOCTL call
   * will not take effect. One shot only. The IOCTL call below is conformal.
   */
#ifdef I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE
  {
    addr_t lAddr;

    createRandomAddress(&lAddr);
    SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
  }
#endif /* I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE */

  /* Keep trying to join (a side effect of successful initialization) until
   * successful. Toggle LEDS to indicate that joining has not occurred.
   */
  while (SMPL_SUCCESS != SMPL_Init(0))
  {
    toggleLED(1);
    toggleLED(2);
    SPIN_ABOUT_A_SECOND; /* calls nwk_pllBackgrounder for us */
  }

  /* LEDs on solid to indicate successful join. */
  if (!BSP_LED2_IS_ON())
  {
    toggleLED(2);
  }
  if (!BSP_LED1_IS_ON())
  {
    toggleLED(1);
  }

  /* Unconditional link to AP which is listening due to successful join. */
  linkTo();

  while (1)
    FHSS_ACTIVE( nwk_pllBackgrounder( false ) );
}

static void linkTo()
{
  uint8_t     msg[2];
  uint8_t     button, misses, done;

  /* Keep trying to link... */
  while (SMPL_SUCCESS != SMPL_Link(&sLinkID1))
  {
    toggleLED(1);
    toggleLED(2);
    SPIN_ABOUT_A_SECOND; /* calls nwk_pllBackgrounder for us */
  }

  /* Turn off LEDs. */
  if (BSP_LED2_IS_ON())
  {
    toggleLED(2);
  }
  if (BSP_LED1_IS_ON())
  {
    toggleLED(1);
  }

#ifndef FREQUENCY_HOPPING
  /* sleep until button press... */
  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);
#endif

  while (1)
  {
    /* keep the FHSS scheduler happy */
    FHSS_ACTIVE( nwk_pllBackgrounder( false ) );
    
    button = 0;
    /* Send a message when either button pressed */
    if (BSP_BUTTON1())
    {  /* calls nwk_pllBackgrounder for us */
      SPIN_ABOUT_A_QUARTER_SECOND;  /* debounce... */
      /* Message to toggle LED 1. */
      button = 1;
    }
    else if (BSP_BUTTON2())
    {  /* calls nwk_pllBackgrounder for us */
      SPIN_ABOUT_A_QUARTER_SECOND;  /* debounce... */
      /* Message to toggle LED 2. */
      button = 2;
    }
    if (button)
    {
      uint8_t      noAck;
      smplStatus_t rc;

#ifndef FREQUENCY_HOPPING
      /* get radio ready...awakens in idle state */
      SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, 0);
#endif

      /* Set TID and designate which LED to toggle */
      msg[1] = ++sTid;
      msg[0] = (button == 1) ? 1 : 2;
      done = 0;
      while (!done)
      {
        noAck = 0;

        /* Try sending message MISSES_IN_A_ROW times looking for ack */
        for (misses=0; misses < MISSES_IN_A_ROW; ++misses)
        {
          if (SMPL_SUCCESS == (rc=SMPL_SendOpt(sLinkID1, msg, sizeof(msg), SMPL_TXOPTION_ACKREQ)))
          {
            /* Message acked. We're done. Toggle LED 1 to indicate ack received. */
            toggleLED(1);
            break;
          }
          if (SMPL_NO_ACK == rc)
          {
            /* Count ack failures. Could also fail becuase of CCA and
             * we don't want to scan in this case.
             */
            noAck++;
          }
        }
        if (MISSES_IN_A_ROW == noAck)
        {
          /* Message not acked. Toggle LED 2. */
          toggleLED(2);
#ifdef FREQUENCY_AGILITY
          /* Assume we're on the wrong channel so look for channel by
           * using the Ping to initiate a scan when it gets no reply. With
           * a successful ping try sending the message again. Otherwise,
           * for any error we get we will wait until the next button
           * press to try again.
           */
          if (SMPL_SUCCESS != SMPL_Ping(sLinkID1))
          {
            done = 1;
          }
#else
          done = 1;
#endif  /* FREQUENCY_AGILITY */
        }
        else
        {
          /* Got the ack or we don't care. We're done. */
          done = 1;
        }
      }

#ifndef FREQUENCY_HOPPING
      /* radio back to sleep */
      SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);
#endif
    }
  }
}


void toggleLED(uint8_t which)
{
  if (1 == which)
  {
    BSP_TOGGLE_LED1();
  }
  else if (2 == which)
  {
    BSP_TOGGLE_LED2();
  }
  return;
}

/*
Todo esto es nuevo
Process the message coming from the UART
*/

void uart_processMessage(void){

if (f_isUartHeaderByte) {
  f_isUartHeaderByte = 0;
  uart_rxFromBuffer(dataPayload,1);
  p_dataPayload++;
}

else{
  // in dataPayload we have: linkID + Command code + message payload
  uart_rxFromBuffer(p_dataPayload,1);
  p_dataPayload++;

  if((p_dataPayload - dataPayload) > (dataPayload[0])){
  f_isUartHeaderByte = 1; // reset flag

  // Process datapayload depending on what type of message it is:
  p_dataPayload = dataPayload;  // re-init pointer
  uint8_t messageLength = *p_dataPayload;
  p_dataPayload++;
  uint8_t destLinkID = *p_dataPayload;  // first byte: destination ID 

  // rest of the message will be the real payload

  if (destLinkID == 0) {  // message for PGN
  // do something:
  pgn_processMessage(p_dataPayload, messageLength);
  p_dataPayload = dataPayload;  // re-init pointer

  }
  else{
    uint8_t i = 0;
    for (i=0;i<sNumCurrentPeers;i++){
      if (destLinkID == sLID[i]){          
      // message for one of the EDs in the network
      // send the message
      rf_sendMessage(p_dataPayload, destLinkID, messageLength); 
      }
    }
  }
    p_dataPayload = dataPayload;  // re-init pointer
     
  }


}

}

/*
Send message to a node with link ID = id
*/

void rf_sendMessage(uint8_t* data, uint8_t id, uint8_t length){
  // we do not send the linkID (dataPayload[0])... increment pointer:
  uint8_t mess = data[1];
  SMPL_Send(id,&mess,sizeof(mess));

}

void sw_init(void){

  p_dataPayload = dataPayload;  // points to the receive frame
  f_isUartHeaderByte = 1; // The first byte will be a header byte
  f_createNetwork = false;

return;

}

/*
Process the message coming from the UART that is destined to the PGN
*/

void pgn_processMessage(uint8_t *data, uint8_t length){

  if (data[1] == 0){
  // code 0: to initialize network
  pgn_createNetwork();
  }
  else if (data[1] == 1){
    /* Send a get description message to all nodes  */
    pgn_requestDescription();
  }
  else {}

}
/*
Send a get description message to all nodes 
*/
void pgn_requestDescription(){
    
    uint8_t i,mess = 0;
    for (i=0; i <sNumCurrentPeers;i++){
    
    SMPL_Send(sLID[i],&mess,sizeof(mess));

    }

}

/*
When this function is called, the PGN will be able to join
and link to EDs
*/

void pgn_createNetwork(void){
  f_createNetwork = true;
}