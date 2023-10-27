/*
 * FibreOpticHAL.h
 *
 *  Created on: Sep 24, 2023
 *      Author: Jarren Lange
 */

#ifndef SRC_FIBREOPTICHAL_C_
#define SRC_FIBREOPTICHAL_C_

#include "FibreOpticHAL.h"
#include <stdio.h>


//Local Variables

//Velocity dependant on the channel.
uint8_t	 chA1Velocity;
uint8_t	 chA2Velocity;
uint8_t	 chB1Velocity;
uint8_t	 chB2Velocity;

//Timeout detection
uint64_t lastCHA1Time;
uint64_t lastCHA2Time;
uint64_t lastCHB1Time;
uint64_t lastCHB2Time;
uint64_t tim1_count;


uint32_t fire_duration_CHA;
uint32_t dwell_duration_CHA;
uint32_t dwell_duration_CHB;
uint32_t fire_duration_CHB;

uint8_t dwell_block_CHA;//Indicator to interupt code that there is a block on the FO channel.
uint8_t dwell_block_CHB;//Indicator to interupt code that there is a block on the FO channel.


//GPIO Pins and port definitions.
//For detecting when the FO have been pulled to GND, such that firing is possible
GPIO_TypeDef* FO_OUTPUT_EN_FBA_GPIO_Port;
uint16_t FO_OUTPUT_EN_FBA_Pin;
GPIO_TypeDef* FO_OUTPUT_EN_FBB_GPIO_Port;
uint16_t FO_OUTPUT_EN_FBB_Pin;

//For enabling optional SOLO pull down transistors
GPIO_TypeDef* SOLO_EN_CHA_GPIO_Port;
uint16_t SOLO_EN_CHA_Pin;
GPIO_TypeDef* SOLO_EN_CHB_GPIO_Port;
uint16_t SOLO_EN_CHB_Pin;

/*
 * Midi commands refer to a velocity to define a volume,
 * we scale the output actual pulse time, to the maximum defined
 * by the user
 */
uint32_t getVelocityAdjustedOnDuration (uint8_t velocity,uint32_t f_dur){
	float factor = velocity/MAX_VELOCITY;
	if(factor>0&&factor<=1){

		uint32_t fireDuration = (uint32_t)(factor*f_dur);
		return fireDuration;
	}else {return 0;}

}
void setCHAActivePeriod(float tDelay_us)
{
	if(tDelay_us<MAX_TIM3_PERIOD_US){
		fire_duration_CHA = (uint32_t)((tDelay_us)* (TIM3_delayuSGain));
	}
}
void setCHADwellPeriod(float tDelay_us)
{
	if(tDelay_us<MAX_TIM3_PERIOD_US){
		dwell_duration_CHA = (uint32_t)(tDelay_us * TIM3_delayuSGain);
	}
}

void setCHBActivePeriod(float tDelay_us)
{
	if(tDelay_us<MAX_TIM4_PERIOD_US){
		fire_duration_CHB = (uint32_t)((tDelay_us)* (TIM4_delayuSGain));
	}
}
void setCHBDwellPeriod(float tDelay_us)
{
	if(tDelay_us<MAX_TIM4_PERIOD_US){
		dwell_duration_CHB = (uint32_t)(tDelay_us * TIM4_delayuSGain);
	}
}

void fireTIM3(uint8_t velocity)
{
	dwell_block_CHA=CH_FIRE;//Block any further firing commands.

	uint32_t fireDuration = getVelocityAdjustedOnDuration(velocity, fire_duration_CHA);

	TIM3->ARR = fireDuration + dwell_duration_CHA;//Set the total time period
	TIM3->CCR1 = fireDuration;//Set the period of on time
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
}

void fireTIM4(uint8_t velocity)
{
	dwell_block_CHB=CH_FIRE;//Block any further firing commands.

	uint32_t fireDuration_B = getVelocityAdjustedOnDuration(velocity, fire_duration_CHB);

	TIM4->ARR = fireDuration_B + dwell_duration_CHB;//Set the total time period
	TIM4->CCR3 = fireDuration_B;//Set the period of on time//Channel 3
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
}
void fireCHA(uint8_t chNo){
	//Choose the velocity based on the channel number.
	uint8_t velocity = chA1Velocity;
	if(chNo == 2)
	{
		velocity = chA2Velocity;
	}
	//Make sure the user wants to fire
	if(isFiringActiveCHA()){
		//Ensure we arent in a dwell time
		if(dwell_block_CHA == CH_FREE&&velocity!=0){
			fireTIM3(velocity);
		}else
		{
			//Tried to run when blocked, do nothing here.
		}
	}
}
void fireCHB(uint8_t chNo){
	//Choose the velocity based on the channel number.
	uint8_t velocity = chB1Velocity;
	if(chNo == 2)
	{
		velocity = chB2Velocity;
	}
	//Make sure the user wants to fire
	if(isFiringActiveCHB()){
		//Ensure we arent in a dwell time
		if(dwell_block_CHB == CH_FREE&&velocity!=0){
			fireTIM4(velocity);
		}else
		{
			//Tried to run when blocked, do nothing here.
		}
	}
}

void setCHAFreq1wVelocity(uint16_t freq, uint8_t velocity)
{
	//If we are actively firing the channel, and the frequency is not 0
	if((freq!= 0)&&isFiringActiveCHA()){
		HAL_TIM_Base_Start_IT(&htim2);
		TIM2->ARR = (uint32_t)( TIM2_freqHzGain /freq);;//Set the period
		chA1Velocity = velocity;
		if(velocity>80){
		}

	}else{
		//Stop the timer
		HAL_TIM_Base_Stop_IT(&htim2);
	}
	//Update the timeout
    lastCHA1Time = tim1_count;
}
void setCHAFreq2wVelocity(uint16_t freq, uint8_t velocity)
{
	//If we are actively firing the channel, and the frequency is not 0
	if((freq!= 0)&&isFiringActiveCHA()){
		//Start the timer
		HAL_TIM_Base_Start_IT(&htim10);
		TIM10->ARR = (uint32_t)( TIM10_freqHzGain /freq);//Set the period
		chA2Velocity = velocity;
	}
	else{
		HAL_TIM_Base_Stop_IT(&htim10);
	}
	//Update the timeout
   lastCHA2Time = tim1_count;
}

void setCHBFreq1wVelocity(uint16_t freq, uint8_t velocity)
{
	//If we are actively firing the channel, and the frequency is not 0
	if((freq!= 0)&&isFiringActiveCHB()){
		TIM5->ARR = (uint32_t)( TIM5_freqHzGain /freq);//Set the period
		chB1Velocity = velocity;
		HAL_TIM_Base_Start_IT(&htim5);

	}else{
		HAL_TIM_Base_Stop_IT(&htim5);
	}
	//Update the timeout
    lastCHB1Time = tim1_count;
}
void setCHBFreq2wVelocity(uint16_t freq, uint8_t velocity)
{
	//If we are actively firing the channel, and the frequency is not 0
	if((freq!= 0)&&isFiringActiveCHB()){
		//Start the timer
		HAL_TIM_Base_Start_IT(&htim11);
		uint32_t val = (uint32_t)( TIM11_freqHzGain /freq);
		TIM11->ARR = val;//Set the period
		chB2Velocity = velocity;
	}
	else{
		HAL_TIM_Base_Stop_IT(&htim11);
	}
	//Update the timeout
   lastCHB2Time = tim1_count;
}


void setCHAFreq1(float freq)
{
	setCHAFreq1wVelocity(freq,DEFAULT_VELOCITY);
}
void setCHAFreq2(float freq)
{
	setCHAFreq2wVelocity(freq,DEFAULT_VELOCITY);
}
void setCHBFreq1(float freq)
{
	setCHBFreq1wVelocity(freq,DEFAULT_VELOCITY);
}
void setCHBFreq2(float freq)
{
	setCHBFreq2wVelocity(freq,DEFAULT_VELOCITY);
}
void defineFiringActiveFBPins(GPIO_TypeDef* PortCHA, uint16_t pinCHA,GPIO_TypeDef* PortCHB, uint16_t pinCHB )
{
	FO_OUTPUT_EN_FBA_GPIO_Port = PortCHA;
	FO_OUTPUT_EN_FBB_GPIO_Port = PortCHB;
	FO_OUTPUT_EN_FBA_Pin = pinCHA;
	FO_OUTPUT_EN_FBB_Pin = pinCHB;
}
void defineSoloEnablePins(GPIO_TypeDef* PortCHA, uint16_t pinCHA,GPIO_TypeDef* PortCHB, uint16_t pinCHB )
{
	SOLO_EN_CHA_GPIO_Port = PortCHA;
	SOLO_EN_CHB_GPIO_Port = PortCHB;
	SOLO_EN_CHA_Pin = pinCHA;
	SOLO_EN_CHB_Pin = pinCHB;
}

uint8_t isFiringActiveCHA(){
	return HAL_GPIO_ReadPin(FO_OUTPUT_EN_FBA_GPIO_Port, FO_OUTPUT_EN_FBA_Pin)==0;
}
uint8_t isFiringActiveCHB(){
	return HAL_GPIO_ReadPin(FO_OUTPUT_EN_FBB_GPIO_Port, FO_OUTPUT_EN_FBB_Pin)==0;
}
void setCHASoloEnable(uint8_t state){
	HAL_GPIO_WritePin(SOLO_EN_CHA_GPIO_Port, SOLO_EN_CHA_Pin, state);
}
void setCHBSoloEnable(uint8_t state){
	HAL_GPIO_WritePin(SOLO_EN_CHB_GPIO_Port, SOLO_EN_CHB_Pin, state);
}

#endif /* SRC_FIBREOPTICHAL_C_ */
