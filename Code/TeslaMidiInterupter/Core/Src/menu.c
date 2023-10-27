/*
 * menu.c
 *
 *  Created on: Aug 22, 2022
 *      Author: Jarren Lange
 */
#include "Menu.h"

#include <stdio.h>

uint8_t currentMenuNO;
uint8_t currentSubMenuNO;
uint8_t subMenuEditParam;


FO_SystemParams FO_CHA_Params;
FO_SystemParams FO_CHB_Params;

void initMenu(){
	//Default Params
	FO_CHA_Params.FixedFrequency_1 = 100;
	FO_CHA_Params.FixedFrequency_2 = 0;
	FO_CHA_Params.midiCh1 = 1;
	FO_CHA_Params.midiCh2 = 0;
	FO_CHA_Params.onTime = 20;
	FO_CHA_Params.dwellTime = 200;
	FO_CHA_Params.enSolo = 0;

	FO_CHB_Params.FixedFrequency_1 = 100;
	FO_CHB_Params.FixedFrequency_2 = 0;
	FO_CHB_Params.midiCh1 = 2;
	FO_CHB_Params.midiCh2 = 3;
	FO_CHB_Params.onTime = 20;
	FO_CHB_Params.dwellTime = 200;
	FO_CHB_Params.enSolo = 0;


}

void menuEncoderRotate(int dir){
	if(subMenuEditParam==0)
	{
		advanceSubMenu(dir);
	}else{
		advanceParam(dir);
	}

}
void advanceParam(int dir)
{
	switch(currentMenuNO){//Fixed Frequency
	case 0:

		switch(currentSubMenuNO){
			case 0:
				advanceF1(&FO_CHA_Params,dir);
				break;
			case 1:
				advanceF2(&FO_CHA_Params,dir);
				break;
			case 2:
				advanceDel(&FO_CHA_Params,dir,A_SIDE_DELAY);
				break;
			case 3:
				advanceDwell(&FO_CHA_Params,dir);
				break;
			case 4:
				advanceSolo(&FO_CHA_Params);
				break;
			case 5:
				advanceF1(&FO_CHB_Params,dir);
				break;
			case 6:
				advanceF2(&FO_CHB_Params,dir);
				break;
			case 7:
				advanceDel(&FO_CHB_Params,dir,B_SIDE_DELAY);
				break;
			case 8:
				advanceDwell(&FO_CHB_Params,dir);
				break;
			case 9:
				advanceSolo(&FO_CHB_Params);
				break;
			default:
				break;
			}
		break;

	case 1:
		switch(currentSubMenuNO){
		case 0:
			advanceCH1(&FO_CHA_Params,dir);
			break;
		case 1:
			advanceCH2(&FO_CHA_Params,dir);
			break;
		case 2:
			advanceDel(&FO_CHA_Params,dir,A_SIDE_DELAY);
			break;
		case 3:
			advanceDwell(&FO_CHA_Params,dir);
			break;
		case 4:
			advanceSolo(&FO_CHA_Params);
			break;
		case 5:
			advanceCH1(&FO_CHB_Params,dir);
			break;
		case 6:
			advanceCH2(&FO_CHB_Params,dir);
			break;
		case 7:
			advanceDel(&FO_CHB_Params,dir,B_SIDE_DELAY);
			break;
		case 8:
			advanceDwell(&FO_CHB_Params,dir);
			break;
		case 9:
			advanceSolo(&FO_CHB_Params);
			break;
			default:
				break;
		}
		break;

		default:
			break;

	}
}
int limitValue(int val, int min, int max){
	if(val>max){
		val=max;
	}else if(val<min){
		val=min;
	}
	return val;
}



void advanceF1(FO_SystemParams* pParams, int dir){
	(*pParams).FixedFrequency_1+= dir*FREQ_STEP;
	(*pParams).FixedFrequency_1 = limitValue((*pParams).FixedFrequency_1,FREQ_MIN,FREQ_MAX);
}
void advanceF2(FO_SystemParams* pParams,int dir){
	(*pParams).FixedFrequency_2+= dir*FREQ_STEP;
	(*pParams).FixedFrequency_2 = limitValue((*pParams).FixedFrequency_2,FREQ_MIN,FREQ_MAX);
}
void advanceDel(FO_SystemParams* pParams,int dir, int side){
	(*pParams).onTime+= dir*FO_DEL_STEP_SIZE;
	if(side == A_SIDE_DELAY){
		(*pParams).onTime = limitValue((*pParams).onTime,FO_MIN_DEL,FO_MAX_DEL_A);
	}else{
		(*pParams).onTime = limitValue((*pParams).onTime,FO_MIN_DEL,FO_MAX_DEL_B);
	}
}
void advanceDwell(FO_SystemParams* pParams,int dir){
	(*pParams).dwellTime+= dir*FO_DWELL_STEP_SIZE;
	(*pParams).dwellTime = limitValue((*pParams).dwellTime,FO_MIN_DWELL,FO_MAX_DWELL);
}
void advanceCH1(FO_SystemParams* pParams,int dir)
{
	(*pParams).midiCh1+=dir;
	if((*pParams).midiCh1>MIDI_MAX_CH){
		(*pParams).midiCh1 = 0;
	}
}
void advanceCH2(FO_SystemParams* pParams,int dir)
{
	(*pParams).midiCh2+=dir;
	if((*pParams).midiCh2>MIDI_MAX_CH){
		(*pParams).midiCh2 = 0;
	}
}
void advanceSolo(FO_SystemParams* pParams)
{
	(*pParams).enSolo = (*pParams).enSolo^0x01;
}



void menuButPush(void){
	subMenuEditParam ++;
	if(subMenuEditParam>1){subMenuEditParam=0;}
}
void menuButDoublePush(void)
{
	advanceMenu();
}
void menuButHold(void){}

void advanceSubMenu(int dir){
	currentSubMenuNO+= dir;
	if(currentSubMenuNO==255){
		currentSubMenuNO = maxSubMenus[currentMenuNO]-1;
	}else if(currentSubMenuNO == maxSubMenus[currentMenuNO]){
		currentSubMenuNO = 0;
	}
}
void advanceMenu(void){
	currentMenuNO++;
	if(MAX_MENU_NO==currentMenuNO)
	{
		currentMenuNO=0;
	}

	currentSubMenuNO = 0;

	if(currentMenuNO != FIXED_F_MENU_NO)
	{
		shutdownFixedFreq();
	}
	printf("Menu adv: %d",currentMenuNO);
}

char * currentMenuText(void)
{
	switch(currentMenuNO){
	case 0:
		return "Fixed Freq     ";
		break;

	case 1:
		return "Midi            ";
		break;

	default:
		return "Default Menu    ";

		break;

	}
}

char * currentSubMenuText(void)
{
	switch(currentMenuNO){
	case 0:

		switch(currentSubMenuNO){
			case 0:
				return "Ca Freq1(Hz):";
				break;
			case 1:
				return "Ca Freq2(Hz):";
				break;
			case 2:
				return "Ca OnTim(us)";
				break;
			case 3:
				return "Ca Dwell(us)";
				break;
			case 4:
				return "Ca Solo (en)";
				break;
			case 5:
				return "Cb Freq1(Hz):";
				break;
			case 6:
				return "Cb Freq2(Hz):";
				break;
			case 7:
				return "Cb OnTim(us)";
				break;
			case 8:
				return "Cb Dwell(us)";
				break;
			case 9:
				return "Cb Solo (en)";
				break;
			default:
				return "DEF";
				break;
			}
		break;

	case 1:
		switch(currentSubMenuNO){
			case 0:
				return "Ca Channel 1:";
				break;
			case 1:
				return "Ca Channel 2";
				break;
			case 2:
				return "Ca OnTim(us)";
				break;
			case 3:
				return "Ca Dwell(us)";
				break;
			case 4:
				return "Ca Solo (en)";
				break;
			case 5:
				return "Cb Channel 1:";
				break;
			case 6:
				return "Cb Channel 2";
				break;
			case 7:
				return "Cb OnTim(us)";
				break;
			case 8:
				return "Cb Dwell(us)";
				break;
			case 9:
				return "Cb Solo (en)";
				break;
			default:
				return "DEF";
				break;
		}
		break;

	default:
		return "DEF";

		break;

	}
}

int currentMenuParameter(){
	switch(currentMenuNO){
	case 0:

		switch(currentSubMenuNO){
		case 0:
			return (int)(FO_CHA_Params.FixedFrequency_1);
			break;
		case 1:
			return (int)(FO_CHA_Params.FixedFrequency_2);
			break;
		case 2:
			return FO_CHA_Params.onTime;
			break;
		case 3:
			return FO_CHA_Params.dwellTime;
			break;
		case 4:
			return FO_CHA_Params.enSolo;
			break;
		case 5:
			return (int)(FO_CHB_Params.FixedFrequency_1);
			break;
		case 6:
			return (int)(FO_CHB_Params.FixedFrequency_2);
			break;
		case 7:
			return FO_CHB_Params.onTime;
			break;
		case 8:
			return FO_CHB_Params.dwellTime;
			break;
		case 9:
			return FO_CHB_Params.enSolo;
			break;
			default:
				return -1;
				break;
			}
		break;

	case 1://MIDI menu
		switch(currentSubMenuNO){
		case 0:
			return (int)(FO_CHA_Params.midiCh1);
			break;
		case 1:
			return (int)(FO_CHA_Params.midiCh2);
			break;
		case 2:
			return FO_CHA_Params.onTime;
			break;
		case 3:
			return FO_CHA_Params.dwellTime;
			break;
		case 4:
			return FO_CHA_Params.enSolo;
			break;
		case 5:
			return (int)(FO_CHB_Params.midiCh1);
			break;
		case 6:
			return (int)(FO_CHB_Params.midiCh2);
			break;
		case 7:
			return FO_CHB_Params.onTime;
			break;
		case 8:
			return FO_CHB_Params.dwellTime;
			break;
		case 9:
			return FO_CHB_Params.enSolo;
			break;
			default:
				return -1;
				break;
		}
		break;

	default:
		return -1;

		break;

	}
}
