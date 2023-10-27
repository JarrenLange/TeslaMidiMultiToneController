/*
 * midi.h
 *  https://newt.phys.unsw.edu.au/jw/notes.html
 *  Created on: 26 Aug 2022
 *      Author: Jarren Lange
 */

#ifndef INC_MIDI_H_
#define INC_MIDI_H_

#include "stm32f4xx_hal.h"
#include "Menu.h"
#define MIN_MIDI_FREQ	21 //A0
#define MAX_MIDI_FREQ	84 //C6

#define MIDI_NOTE_ON	0x90
#define MIDI_NOTE_OFF	0x80

//#define PRINT_FREQS_ON_INIT


void populateFreqs();

void initMIDI(void);

uint16_t freqFromMIDINote(uint8_t note);

extern uint8_t serial_buffer[64]; //Populated by CDC_Receive_FS in file USB_DEVICE/App/usbd_cdc_if.c
extern uint8_t *pCHA1;
extern uint8_t *pCHA2;
extern uint8_t *pCHB1;
extern uint8_t *pCHB2;
extern uint8_t *pMIDIEn;
extern uint16_t midi_freqs[MAX_MIDI_FREQ-MIN_MIDI_FREQ];


void processMIDIOnCMD(uint8_t, uint8_t, uint8_t);
void processMIDIOffCMD(uint8_t);

void registerMIDIEnable(uint8_t*);

extern void setCHAFreq1(float);
extern void setCHAFreq1(float);
extern void setCHBFreq1wVelocity(uint16_t, uint8_t);
extern void setCHBFreq2wVelocity(uint16_t, uint8_t);
extern void setCHBFreq1(float);
extern void setCHBFreq1(float);
extern void setCHBFreq1wVelocity(uint16_t, uint8_t);
extern void setCHBFreq2wVelocity(uint16_t, uint8_t);

void registerMidiChannels(uint8_t*,uint8_t*,uint8_t*,uint8_t*);


#endif /* INC_MIDI_H_ */
