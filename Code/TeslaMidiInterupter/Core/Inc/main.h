/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "Menu.h"
#include "midi.h"
#include "FibreOpticHAL.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BuiltInLED_Pin GPIO_PIN_13
#define BuiltInLED_GPIO_Port GPIOC
#define LCD_D3_Pin GPIO_PIN_2
#define LCD_D3_GPIO_Port GPIOA
#define LCD_D2_Pin GPIO_PIN_3
#define LCD_D2_GPIO_Port GPIOA
#define LCD_D1_Pin GPIO_PIN_4
#define LCD_D1_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_5
#define LCD_D0_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_6
#define LCD_E_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_7
#define LCD_RS_GPIO_Port GPIOA
#define FO_OUTPUT_EN_FB1_Pin GPIO_PIN_0
#define FO_OUTPUT_EN_FB1_GPIO_Port GPIOB
#define FO_OUTPUT_EN_FB2_Pin GPIO_PIN_1
#define FO_OUTPUT_EN_FB2_GPIO_Port GPIOB
#define SOLO_EN_CH1_Pin GPIO_PIN_2
#define SOLO_EN_CH1_GPIO_Port GPIOB
#define SOLO_EN_CH2_Pin GPIO_PIN_15
#define SOLO_EN_CH2_GPIO_Port GPIOB
#define LCD_Backlight_PWM_Pin GPIO_PIN_8
#define LCD_Backlight_PWM_GPIO_Port GPIOA
#define FO_OUTPUT_CH1_Pin GPIO_PIN_4
#define FO_OUTPUT_CH1_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOB
#define ENC_A_EXTI_IRQn EXTI9_5_IRQn
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOB
#define ENC_B_EXTI_IRQn EXTI9_5_IRQn
#define FO_OUTPUT_CH2_Pin GPIO_PIN_8
#define FO_OUTPUT_CH2_GPIO_Port GPIOB
#define ENC_BUT_Pin GPIO_PIN_9
#define ENC_BUT_GPIO_Port GPIOB
#define ENC_BUT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

void interuptCH1(void);



extern uint64_t tim1_count;

extern uint8_t dwell_block_CH1;
extern uint32_t fire_duration_CH1;
extern uint32_t dwell_duration_CH1;

extern uint8_t encoderPrev;
extern int8_t encDir;
extern uint64_t lastEncTime;
extern uint64_t lastButTime;
extern uint64_t lastDoubleTime;
extern uint64_t lastDispTime;
extern uint64_t lastDispResTime;
extern uint64_t lastCH1ATime;
extern uint64_t lastCH1BTime;

//extern uint8_t	 chA1Velocity;
//extern uint8_t	 chA2Velocity;
//extern uint8_t	 chB1Velocity;
//extern uint8_t	 chB2Velocity;

extern uint8_t 	 encPos;

extern struct Lcd_HandleTypeDef lcd;

void updateDisplay();
void updateDisplayFeedback();
void updateHardware();
void checkChannelTimeouts();

#define CH1_Pin GPIO_PIN_0
#define CH1_Port GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
