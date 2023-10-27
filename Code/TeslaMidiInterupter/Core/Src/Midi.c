/*
 * Midi.c
 *
 *  Created on: 26 Aug 2022
 *      Author: Jarren Lange
 */

#include "midi.h"
#include <math.h>


uint8_t serial_buffer[64]; //Populated by CDC_Receive_FS in file USB_DEVICE/App/usbd_cdc_if.c
uint8_t *pCHA1;
uint8_t *pCHA2;
uint8_t *pCHB1;
uint8_t *pCHB2;
uint8_t *pMIDIEn;
uint16_t midi_freqs[MAX_MIDI_FREQ-MIN_MIDI_FREQ];

void populateFreqs()
{
	for(int i = MIN_MIDI_FREQ; i<MAX_MIDI_FREQ;i++){
		midi_freqs[i-MIN_MIDI_FREQ] = (uint16_t) (pow(2.0,(i-69.0)/12)*440);
#ifdef PRINT_FREQS_ON_INIT
		printf("No %d, FREQ: %d  Hz \n",i,midi_freqs[i-MIN_MIDI_FREQ]);
		HAL_Delay(2);
#endif
	}
}

void initMIDI(void){
	populateFreqs();
}

uint16_t freqFromMIDINote(uint8_t note)
{
	if(note >= MIN_MIDI_FREQ&&note< MAX_MIDI_FREQ)
	{
		return midi_freqs[note-MIN_MIDI_FREQ];
	}
	return 0;
}

void serial_received(uint8_t len)
{
	//printf("%d \n",len);
	if(len == 3&&((*pMIDIEn))){
		uint8_t cmd_val = serial_buffer[0];
		uint8_t pitch = serial_buffer[1];
		uint8_t velocity = serial_buffer[2];

		if((cmd_val&0xF0) == MIDI_NOTE_ON)
		{
			uint8_t channel = cmd_val&0x0F;

			processMIDIOnCMD(channel, pitch, velocity);
		}
		if((cmd_val&0xF0) == MIDI_NOTE_OFF)
		{
			uint8_t channel = cmd_val&0x0F;

			processMIDIOffCMD(channel);
		}
	}
}

void registerMIDIEnable(uint8_t *pEn)
{
	pMIDIEn = pEn;
}
void processMIDIOnCMD(uint8_t channel, uint8_t pitch, uint8_t velocity)
{
	if((*pCHA1) == channel)
	{
		setCHAFreq1wVelocity(freqFromMIDINote(pitch),velocity);
	}else if((*pCHA2) == channel)
	{
		setCHAFreq2wVelocity(freqFromMIDINote(pitch),velocity);
	}
	//Second Tesla
	if((*pCHB1) == channel)
	{
		setCHBFreq1wVelocity(freqFromMIDINote(pitch),velocity);
	}else if((*pCHB2) == channel)
	{
		setCHBFreq2wVelocity(freqFromMIDINote(pitch),velocity);
	}
}
void processMIDIOffCMD(uint8_t channel)
{
	if((*pCHA1) == channel)
	{
		setCHAFreq1(0);
	}
	if((*pCHA2) == channel)
	{
		setCHAFreq2(0);
	}
	if((*pCHB1) == channel)
	{
		setCHBFreq1(0);
	}
	if((*pCHB2) == channel)
	{
		setCHBFreq2(0);
	}
}

void registerMidiChannels(uint8_t *p1,uint8_t *p2,uint8_t *p3,uint8_t *p4)
{
	pCHA1 = p1;
	pCHA2 = p2;
	pCHB1 = p3;
	pCHB2 = p4;
}
