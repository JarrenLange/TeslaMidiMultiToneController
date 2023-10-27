/*
 * Menu.h
 *
 *  Created on: Aug 22, 2022
 *      Author: Jarren Lange
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"

#define MAX_MENU_NO		2
#define FIXED_F_MENU_NO	0

#define MIDI_MAX_CH		20
#define FO_MAX_DEL_A	50
#define FO_MAX_DEL_B	80
#define B_SIDE_DELAY	1
#define A_SIDE_DELAY	0

#define FO_DEL_STEP_SIZE	2
#define FO_MIN_DEL		6
#define FO_MAX_DWELL	990
#define FO_DWELL_STEP_SIZE	10
#define FO_MIN_DWELL		200

#define FREQ_MIN		0
#define FREQ_MAX		990
#define FREQ_STEP		10

void initMenu();

char * currentMenuText(void);
char * currentMenuSubText(void);
void menuEncoderRotate(int);
void advanceParam(int);

void menuButPush(void);
void menuButDoublePush(void);
void menuButHold(void);

void advanceMenu(void);
void advanceSubMenu(int);


struct FO_SystemParams {
	uint32_t FixedFrequency_1;
	uint32_t FixedFrequency_2;
	uint8_t midiCh1;
	uint8_t midiCh2;
	int onTime;
	int dwellTime;
	uint8_t enSolo;
};
typedef struct FO_SystemParams FO_SystemParams;

//Channel A
void advanceF1(FO_SystemParams*,int);
void advanceF2(FO_SystemParams*,int);
void advanceDel(FO_SystemParams* ,int, int);
void advanceCH1(FO_SystemParams*,int);
void advanceCH2(FO_SystemParams*,int);
void advanceDwell(FO_SystemParams*,int);
void advanceSolo(FO_SystemParams*);

int limitValue(int, int, int);
extern void shutdownFixedFreq();

int currentMenuParameter();
char * currentSubMenuText(void);

extern uint8_t currentMenuNO;
extern uint8_t currentSubMenuNO;
extern uint8_t subMenuEditParam;

extern FO_SystemParams FO_CHA_Params;
extern FO_SystemParams FO_CHB_Params;

static const int maxSubMenus[2] = {10, 10};



#endif /* INC_MENU_H_ */
