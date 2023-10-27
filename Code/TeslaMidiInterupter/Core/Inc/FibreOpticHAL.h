/*
 * FibreOpticHAL.h
 *
 *  Created on: Sep 24, 2023
 *      Author: Jarren Lange
 */

#ifndef INC_FIBREOPTICHAL_H_
#define INC_FIBREOPTICHAL_H_
#include "stm32f4xx_hal.h"

//Hardware handles
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;


//Gains used to define Hardware to external characteristics
#define TIM3_BASE_DELAY_OFFSET		5
#define TIM3_delayuSGain	48
#define TIM2_freqHzGain 	96000000
#define TIM10_freqHzGain 	96000000/(15+1) //(Clk/Prescale+1)


//Second Tesla Hardware characteristics
#define TIM5_freqHzGain 	96000000
#define TIM4_delayuSGain	48
#define TIM11_freqHzGain 	96000000/(15+1) //(Clk/Prescale+1)
#define TIM_16bit_MIN_FREQ 	96000000/(16384*(15+1)) //(Clk/Prescale+1)

#define CH_FREE	0
#define CH_FIRE	1
#define CH_DWELL 2

#define EN_DWELL
#define DEFAULT_DWELL	200

#define MIDI_FREQ_TIMEOUT	1000

#define MAX_TIM3_PERIOD_US	(65535/(48*(1+1)))
#define MAX_TIM4_PERIOD_US	(65535/(48*(1+1)))

#define MAX_VELOCITY	127.0
#define DEFAULT_VELOCITY	126

//

void fireCHA(uint8_t);
void fireCHB(uint8_t);

//Individual functions for the first Telsa Channel
void setCHAFreq1(float);
void setCHAFreq2(float);
void setCHAFreq1wVelocity(uint16_t, uint8_t);
void setCHAFreq2wVelocity(uint16_t, uint8_t);
void setCHADwellPeriod(float);
void setCHASoloEnable(uint8_t);
void setCHAActivePeriod(float);


//Individual functions for the second Telsa Channel
void setCHBFreq1(float);
void setCHBFreq2(float);
void setCHBFreq1wVelocity(uint16_t, uint8_t);
void setCHBFreq2wVelocity(uint16_t, uint8_t);
void setCHBDwellPeriod(float);
void setCHBSoloEnable(uint8_t);
void setCHBActivePeriod(float);

void defineFiringActiveFBPins(GPIO_TypeDef* PortCHA, uint16_t pinCHA,GPIO_TypeDef* PortCHB, uint16_t pinCHB );
void defineSoloEnablePins(GPIO_TypeDef* PortCHA, uint16_t pinCHA,GPIO_TypeDef* PortCHB, uint16_t pinCHB );
uint8_t isFiringActiveCHA();
uint8_t isFiringActiveCHB();

void fireTIM3(uint8_t);
void fireTIM4(uint8_t);

#endif /* INC_FIBREOPTICHAL_H_ */
