// Header:
// File Name: midi.c
// Author:    Pietro Lorefice
// Date:      10/18/2014

#include "midi.h"
#include "usb_device.h"

void MIDI_SendMsg (uint8_t type, uint8_t note, uint8_t velocity)
{
  uint8_t midi_msg[4];

  midi_msg[0] = 9;
  midi_msg[1] = type;
  midi_msg[2] = note;
  midi_msg[3] = velocity;
      
  USBD_LL_Transmit(&hUsbDeviceFS, MIDI_IN_EP, (uint8_t*)&midi_msg, 4);
}
