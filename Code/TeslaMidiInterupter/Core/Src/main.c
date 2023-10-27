/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Jarren Lange
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the Github repository
  *
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
	Function Description:
	Following a valid MIDI command from the USB/serial port or a constant frequency
	We get the frequency and velocity (volume) from the MIDI command.
	We set the one frequency of a tone generating timer to this frequency.
	On one of these tone generating timers being triggered, we call the firing command.
	The firing command uses a timer to generate a period with a variable on and off command.
	During firing the timer cannot be reset.

	After the defined on and off time of the firing timer, we wait for the next frequency trigger.

	For Musical Channel 1 (so Tesla coil 1):
		TIM3 	- 	Firing Channel
		TIM2 	- 	Frequency CH1A
		TIM10 	- 	Frequency CH1B

	For Musical Channel 2 (So Tesla coil 2):
		TIM4	- 	Firing Channel (CH3)
		TIM5	-	Frequency CH2A
		TIM11	-	Frequency CH2B

	Timer 1 constantly runs in the background to provide a timeout reference point
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

uint8_t encoderPrev;
uint8_t encPos;
int8_t encDir;


uint64_t lastEncTime;
uint64_t lastButTime;
uint64_t lastDoubleTime;
uint64_t lastDispTime;
uint64_t lastDispResTime;

//Timeouts for the MIDI processes, to timeout a MIDI tone
//Because sometimes the Micro misses a MIDI command to turn off

extern uint64_t lastCHA1Time;
extern uint64_t lastCHA2Time;
extern uint64_t lastCHB1Time;
extern uint64_t lastCHB2Time;
extern uint8_t dwell_block_CHA;
extern uint8_t dwell_block_CHB;

struct Lcd_HandleTypeDef lcd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  //HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_Base_Start_IT(&htim10);
  //HAL_TIM_Base_Start_IT(&htim11);

  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

  Lcd_PortType ports[] = {
		  LCD_D0_GPIO_Port, LCD_D1_GPIO_Port, LCD_D2_GPIO_Port, LCD_D3_GPIO_Port
  };

  Lcd_PinType pins[] = {LCD_D0_Pin, LCD_D1_Pin, LCD_D2_Pin, LCD_D3_Pin};


  lcd = Lcd_create(ports, pins, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_E_GPIO_Port, LCD_E_Pin, LCD_4_BIT_MODE);

  Lcd_cursor(&lcd, 0,0);
  Lcd_string(&lcd, "Tesla FibreOptic");
  Lcd_cursor(&lcd, 1,0);
  Lcd_string(&lcd, "Dual Tesla Control");
  Lcd_cursor(&lcd, 0,16);
  Lcd_string(&lcd, "V1.1");
  Lcd_cursor(&lcd, 1,16);
  Lcd_string(&lcd, "Dr. Jarren Lange");
  DELAY(1000);

  Lcd_cursor(&lcd, 1,2);

	initMenu();
	updateHardware();

	defineFiringActiveFBPins(FO_OUTPUT_EN_FB1_GPIO_Port, FO_OUTPUT_EN_FB1_Pin, FO_OUTPUT_EN_FB2_GPIO_Port,FO_OUTPUT_EN_FB2_Pin);
	defineSoloEnablePins(SOLO_EN_CH1_GPIO_Port, SOLO_EN_CH1_Pin, SOLO_EN_CH2_GPIO_Port, SOLO_EN_CH2_Pin);

	initMIDI();
	registerMidiChannels(&FO_CHA_Params.midiCh1,&FO_CHA_Params.midiCh2,&FO_CHB_Params.midiCh1,&FO_CHB_Params.midiCh2);
	registerMIDIEnable(&currentMenuNO);//Since Fixed menu i zero, we can use this as a hack for the enable
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Started\n");
  while (1)
  {
	  //Update the display every 200ms.
	  if((lastDispTime+200)<tim1_count)
	  {
		  updateDisplay();
		  lastDispTime =tim1_count;
	  }
	  //Reset the display every 4000ms.
	  //This is to counteract the occasional misoperation of the LCD, assumedly due to EMI (no idea on the source)
	  if((lastDispResTime+4000)<tim1_count)
	  {
		  lcd = Lcd_create(ports, pins, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_E_GPIO_Port, LCD_E_Pin, LCD_4_BIT_MODE);
		  lastDispResTime =tim1_count;
		  updateHardware();
	  }
	  //Check the timeouts if we are operating in MIDI mode.
	  if(currentMenuNO!=FIXED_F_MENU_NO)
	  {
		  //When in midi, it can happen that a tone kill signal is missed.
		  //For that reason we check the timeouts to stop a tone after a fixed time period.

		  checkChannelTimeouts();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 512;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 15;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BuiltInLED_Pin|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_D3_Pin|LCD_D2_Pin|LCD_D1_Pin|LCD_D0_Pin
                          |LCD_E_Pin|LCD_RS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SOLO_EN_CH1_Pin|SOLO_EN_CH2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BuiltInLED_Pin */
  GPIO_InitStruct.Pin = BuiltInLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BuiltInLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D3_Pin LCD_D2_Pin LCD_D1_Pin LCD_D0_Pin
                           LCD_E_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_D3_Pin|LCD_D2_Pin|LCD_D1_Pin|LCD_D0_Pin
                          |LCD_E_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FO_OUTPUT_EN_FB1_Pin */
  GPIO_InitStruct.Pin = FO_OUTPUT_EN_FB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FO_OUTPUT_EN_FB1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FO_OUTPUT_EN_FB2_Pin */
  GPIO_InitStruct.Pin = FO_OUTPUT_EN_FB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FO_OUTPUT_EN_FB2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SOLO_EN_CH1_Pin SOLO_EN_CH2_Pin */
  GPIO_InitStruct.Pin = SOLO_EN_CH1_Pin|SOLO_EN_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_A_Pin ENC_B_Pin ENC_BUT_Pin */
  GPIO_InitStruct.Pin = ENC_A_Pin|ENC_B_Pin|ENC_BUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*) ptr, len);
    return len;
}


//Sometimes you need to forcefully shut down a frequency channel if it hasnt been updated in a while
void checkChannelTimeouts(){


	  if((lastCHA1Time+MIDI_FREQ_TIMEOUT)<tim1_count)
	  {
		  setCHAFreq1(0);
	  }
	  if((lastCHA2Time+MIDI_FREQ_TIMEOUT)<tim1_count)
	  {
		  setCHAFreq2(0);
	  }
	  if((lastCHB1Time+MIDI_FREQ_TIMEOUT)<tim1_count)
	  {
		  setCHBFreq1(0);
	  }
	  if((lastCHB2Time+MIDI_FREQ_TIMEOUT)<tim1_count)
	  {
		  setCHBFreq2(0);
	  }
}

void interuptCHA()
{
	HAL_GPIO_WritePin(CH1_Port, CH1_Pin, 0);
	HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
	dwell_block_CHA=CH_FREE;//Allow a firing

}
void interuptCHB()
{
	//HAL_GPIO_WritePin(CH1_Port, CH1_Pin, 0);
	HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_3);
	dwell_block_CHB=CH_FREE;//Allow a firing

}
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 // Check which  timer triggered this callback
	  if (htim == &htim3 )//TIM3 fires fibre optic 1
	  {
		  HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
		  interuptCHA();
	  }
	  if (htim == &htim4 )//TIM4 fires fibre optic 2
	  {
		  HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_3);
		  interuptCHB();
	  }
	if (htim == &htim1 )//TIM1 is for the timeouts
		  {
		   tim1_count++;
		  }
	  if (htim == &htim2 )
	  {
		fireCHA(1);
	  }
	  if (htim == &htim5 )
	  {
		  fireCHB(1);
	  }
	  if (htim == &htim10 )
	  {
		  fireCHA(2);
	  }
	  if (htim == &htim11 )
	  {
		  fireCHB(2);
	  }
}
void HAL_GPIO_EXTI_Callback(uint16_t p)
{
	//For check if the interupt were caused by the rotatary encoder.
	if(p== ENC_A_Pin||p==ENC_B_Pin)
	{
		uint8_t temp = ((HAL_GPIO_ReadPin(ENC_A_GPIO_Port,ENC_A_Pin)<<1)&0x2)  | (HAL_GPIO_ReadPin(ENC_B_GPIO_Port,ENC_B_Pin)) ;
		uint8_t diff = (temp^encoderPrev);
		if(temp==0x02||temp==0x01){
			//part turn
			//Define the directions here.
			if((diff&0x2)==0x2){
				encDir=-1;
			}else{
				encDir=1;
			}
		}else{
			//Trying to detect actual changes in encoder position.
			//In our case a full rotation, which would cause 2 interupts in a short period of time
			if((lastEncTime+100)<tim1_count)
			{
				menuEncoderRotate(encDir);
				updateHardware();
				encPos += encDir;
				lastEncTime=tim1_count;
			}
		}


		encoderPrev= temp;
	}
	//Interupt source is the Encoder button.
	if(p==ENC_BUT_Pin)
	{
		//for the riding edge
		if( HAL_GPIO_ReadPin(ENC_BUT_GPIO_Port,ENC_BUT_Pin)==1)
		{
			//If the last button press happened between 100 ms since the last button press
			if((lastButTime+100)<tim1_count&&(lastDoubleTime+500)<tim1_count)//SinglePress
			{
				menuButPush();
			}
			//If the last button press was less than 500ms ago
		}else if((lastButTime+500)>tim1_count) {
				menuButDoublePush();
				lastDoubleTime = tim1_count;
		}

		lastButTime = tim1_count;
	}
}
void shutdownFixedFreq(){
	//When we switch to MIDI we kill all fixed frequency commands.
	setCHAFreq1(0);
	setCHAFreq2(0);
	setCHBFreq1(0);
	setCHBFreq2(0);
}
void updateDisplay()
{
	Lcd_cursor(&lcd, 0,0);
	Lcd_string(&lcd, currentMenuText());
	Lcd_cursor(&lcd, 1,0);
	Lcd_string(&lcd, currentSubMenuText());
	Lcd_cursor(&lcd, 1,13);
	Lcd_int(&lcd, currentMenuParameter());

	//Indicators of Firing
	Lcd_cursor(&lcd, 0,14);
	if(isFiringActiveCHA()){//Zero is active
		Lcd_string(&lcd, "*");
	}else{
		//display an S if solo is enabled
		if(FO_CHA_Params.enSolo)
		{
			Lcd_string(&lcd, "S");
		}else{
			Lcd_string(&lcd, "O");
		}
	}
	if(isFiringActiveCHB()){//Zero is active
		Lcd_string(&lcd, "*");
	}else{		//display an S if solo is enabled
		if(FO_CHB_Params.enSolo)
		{
			Lcd_string(&lcd, "S");
		}else{
			Lcd_string(&lcd, "O");
		}
	}

	updateDisplayFeedback();
	//Position of the cursor to indicate what is being edited.
	//Menu is indicated by the first character, parameter by the last.
	if(subMenuEditParam==1){
		Lcd_cursor(&lcd, 1,15);
	}else{
		Lcd_cursor(&lcd, 1,0);
	}

}

void updateDisplayFeedback()
{
	if(currentMenuNO == 0){
		//Start of the 3rd Line
		Lcd_cursor(&lcd, 0,16);
		Lcd_string(&lcd, "A");
		Lcd_int(&lcd, FO_CHA_Params.FixedFrequency_1);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHA_Params.FixedFrequency_2);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHA_Params.onTime);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHA_Params.dwellTime);
		//Start of the 4rd Line
		Lcd_cursor(&lcd, 1,16);
		Lcd_string(&lcd, "B");
		Lcd_int(&lcd, FO_CHB_Params.FixedFrequency_1);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHB_Params.FixedFrequency_2);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHB_Params.onTime);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHB_Params.dwellTime);
	}else{
		//Start of the 3rd Line
		Lcd_cursor(&lcd, 0,16);
		Lcd_string(&lcd, "A");
		Lcd_int(&lcd, FO_CHA_Params.midiCh1);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHA_Params.midiCh2);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHA_Params.onTime);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHA_Params.dwellTime);
		//Start of the 4rd Line
		Lcd_cursor(&lcd, 1,16);
		Lcd_string(&lcd, "B");
		Lcd_int(&lcd, FO_CHB_Params.midiCh1);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHB_Params.midiCh2);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHB_Params.onTime);
		Lcd_string(&lcd, "*");
		Lcd_int(&lcd, FO_CHB_Params.dwellTime);
	}

}
void updateHardware(){
	if(currentMenuNO == FIXED_F_MENU_NO)
	{
		//These Params come from the menu.'
		setCHAFreq1(FO_CHA_Params.FixedFrequency_1);
		setCHAFreq2(FO_CHA_Params.FixedFrequency_2);
		setCHAActivePeriod(FO_CHA_Params.onTime);
		setCHADwellPeriod(FO_CHA_Params.dwellTime);
		setCHASoloEnable(FO_CHA_Params.enSolo);
		//These Params come from the menu.'
		setCHBFreq1(FO_CHB_Params.FixedFrequency_1);
		setCHBFreq2(FO_CHB_Params.FixedFrequency_2);
		setCHBActivePeriod(FO_CHB_Params.onTime);
		//setCHBActivePeriod(40);
		setCHBDwellPeriod(FO_CHB_Params.dwellTime);
		setCHBSoloEnable(FO_CHB_Params.enSolo);
	}else
	{
		//These Params come from the menu.
		setCHAActivePeriod(FO_CHA_Params.onTime);
		setCHADwellPeriod(FO_CHA_Params.dwellTime);
		setCHASoloEnable(FO_CHA_Params.enSolo);

		setCHBActivePeriod(FO_CHB_Params.onTime);
		setCHBDwellPeriod(FO_CHB_Params.dwellTime);
		setCHBSoloEnable(FO_CHB_Params.enSolo);
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
