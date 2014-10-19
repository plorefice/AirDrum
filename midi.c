// Header:
// File Name: midi.c
// Author:    Pietro Lorefice
// Date:      10/18/2014

#include "midi.h"
#include "rl_usb.h"                     // Keil.MDK-Pro::USB:CORE

void MIDI_SendMsg (uint8_t type, uint8_t note, uint8_t velocity)
{
	uint8_t midi_msg[3];
	
	midi_msg[0] = type;
	midi_msg[1] = note;
	midi_msg[2] = velocity;
	
	USBD_CDC_ACM_WriteData (0, midi_msg, 3);
}
