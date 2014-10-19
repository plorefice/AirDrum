/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::USB:Device
 * Copyright (c) 2004-2014 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    USBD_User_CDC_0.c
 * Purpose: USB Device Communication Device Class (CDC) User module
 * Rev.:    V6.00
 *----------------------------------------------------------------------------*/
/**
 * \addtogroup usbd_cdcFunctions
 *
 * USBD_User_CDC_0.c implements the application specific functionality of the
 * CDC class and is used to receive and send data to the USB Host.
 *
 * The implementation must match the configuration file USBD_Config_CDC_0.h.  
 * The following values in USBD_Config_CDC_0.h affect the user code:
 *
 *  - 'Maximum Communication Device Send Buffer Size' specifies the maximum 
 *    value for \em len in \ref USBD_CDC_ACM_WriteData
 *
 *  - 'Maximum Communication Device Receive Buffer Size' specifies the maximum 
 *    value for \em len in \ref USBD_CDC_ACM_ReadData
 *
 */


//! [code_USBD_User_CDC] 

#include <stdbool.h>

#include "usb_cdc.h"

// Called during USBD_Initialize to initialize the USB Device class.
void USBD_CDC0_ACM_Initialize (void) {
  // Add code for initialization
}


// Called during USBD_Uninitialize to de-initialize the USB Device class.
void USBD_CDC0_ACM_Uninitialize (void) {
  // Add code for de-initialization
}


// Called upon USB Reset Event
void USBD_CDC0_ACM_Reset (void) {
  // Add code for reset
}


// Called upon USB request to Set Line Coding.
// \param[in]   line_coding   pointer to \ref CDC_LINE_CODING structure.
// \return      true          set line coding request processed. 
// \return      false         set line coding request not supported or not processed.
bool USBD_CDC0_ACM_SetLineCoding (CDC_LINE_CODING *line_coding) {
  // Add code for set line coding

  return true;
}


// Called upon USB request to Get Line Coding.
// \param[out]  line_coding   pointer to \ref CDC_LINE_CODING structure.
// \return      true          get line coding request processed. 
// \return      false         get line coding request not supported or not processed.
bool USBD_CDC0_ACM_GetLineCoding (CDC_LINE_CODING *line_coding) {
  // Add code for get line coding

  return true;
}


// Called upon Set Control Line State request.
// \param [in]  state         control line settings bitmap.
//                - bit 0: DTR state 
//                - bit 1: RTS state
// \return      true          set control line state request processed. 
// \return      false         set control line state request not supported or not processed.
bool USBD_CDC0_ACM_SetControlLineState (uint16_t state) {
  // Add code for set control line state

  return true;
}

//! [code_USBD_User_CDC] 
