// Header:
// File Name: midi.h
// Author:    Pietro Lorefice
// Date:      10/18/2014

#ifndef __MIDI_H
#define __MIDI_H

#include "stm32f4xx_hal.h"

void MIDI_SendMsg (uint8_t type, uint8_t note, uint8_t velocity);

#endif /* __MIDI_H */
