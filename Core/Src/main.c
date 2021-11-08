/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "MAX31855.h"
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float temp1=0;	// Variable to get Temperature 1
float temp2=0;	// Variable to get Temperature 2
float temp3=0;	// Variable to get Temperature 3
float temp4=0;	// Variable to get Temperature 4
uint32_t avetemp=0; //Variable to be printed to 7-segment display
uint32_t c1, c2, c3; //temporary variables for 7-segment display
uint32_t setTemp; //user input temperature
uint8_t counter=0; //counter to set position of roller blind
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define D1_HIGH() HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET)
#define D1_LOW() HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET)
#define D2_HIGH() HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET)
#define D2_LOW() HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET)
#define D3_HIGH() HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET)
#define D3_LOW() HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET)

uint8_t segmentNumber[10] = {
        0x3f,  // 0
        0x06,  // 1
        0x5b,  // 2
        0x4f,  // 3
        0x66,  // 4
        0x6d,  // 5
        0x7d,  // 6
        0x07,  // 7
        0x7f,  // 8
        0x67   // 9
};

void SevenSegment_Update(uint8_t number){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, ((number>>0)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, ((number>>1)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, ((number>>2)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, ((number>>3)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, ((number>>4)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ((number>>5)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, ((number>>6)&0x01));
};

void ADC_select_CH10(void)
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_10;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_select_CH11(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_11;
	  sConfig.Rank = ADC_REGULAR_RANK_2;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_select_CH12(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_12;
	  sConfig.Rank = ADC_REGULAR_RANK_3;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }


}

void ADC_select_CH13(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_13;
	  sConfig.Rank = ADC_REGULAR_RANK_4;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_select_CH0(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = ADC_REGULAR_RANK_5;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

float ADCval[4];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  uint16_t raw;
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);               //Initialize stm32 timer 3
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);  //PB0 Start pwm temp motor 100% duty cycle
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);  //PB0 Start pwm pitch motor 100% duty cycle
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);  //PB0 Start pwm azimuth motor 100% duty cycle


	//------------------------------------------test temp motor--------------------------------------
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,75);
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);   // Start temp motor CCW rotation
	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET); // Start temp motor CCW rotation
	 HAL_Delay(20000);
	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);   // Stop temp motor CCW rotation
	 HAL_Delay(5000);
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);   // Start temp motor CCW rotation
	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET); // Start temp motor CCW rotation
	 HAL_Delay(20000);
	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);   // Stop temp motor CCW rotation
	 HAL_Delay(5000);
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);   // Start temp motor CW rotation
	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET); // Start temp motor CW rotation
	 HAL_Delay(40000);
	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET); // Stop temp motor CW rotation
	//--------------------------------------------end test temp motor----------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { //-----------------------------------------------------------------------------------------read temperatures from thermocouple-------------
	  temp1=Max31855_Read_Temp1(); 			// Gets Temperature 1 from MAX31855
	  temp2=Max31855_Read_Temp2(); 			// Gets Temperature 2 from MAX31855
	  temp3=Max31855_Read_Temp3(); 			// Gets Temperature 3 from MAX31855
	  temp4=Max31855_Read_Temp4(); 			// Gets Temperature 4 from MAX31855
	  avetemp = (temp1+temp2+temp3+temp4)/4; //determine average temperature from 4 MAX31855 units to be displayed on 7-segment display
	  //--------------------------------------------------------------------------------------end read temperatures------------------------------

	  //---------------------------------------------------------------------------Display Temp on LED------------------------------
	  c1 = (avetemp/100)%10; //first digit
	  c2 = (avetemp/10)%10; //second digit
	  c3 = avetemp%10; //third digit

	  	  SevenSegment_Update(segmentNumber[c1]); //note common cathode 7-segment display therefore must pull D1, D2 and D3 from low to high to write to the display
	  	  D1_LOW();
	  	  HAL_Delay(7);
	  	  D1_HIGH();

	  	  SevenSegment_Update(segmentNumber[c2]);
	  	  D2_LOW();
	  	  HAL_Delay(7);
	  	  D2_HIGH();

	  	  SevenSegment_Update(segmentNumber[c3]);
	  	  D3_LOW();
	  	  HAL_Delay(7);
	  	  D3_HIGH();
	  //--------------------------------------------------------------------------End of LED display-------------------------------

	  //-----------------------------------------------------------------------------read user input from potentiometer------
	   ADC_select_CH10();
	   HAL_ADC_Start(&hadc1);
	   HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	   raw = HAL_ADC_GetValue(&hadc1);
	   HAL_ADC_Stop(&hadc1);

	   //the following are the 10 temperature intervals the device can be set to by the user
	   if(raw > 0 && raw < 409.5)
	   {
		   setTemp = 45;
	   }
	   else if(raw > 409.5 && raw < 819)
	   {
		   setTemp = 60;
	   }
	   else if(raw > 819 && raw < 1228.5)
	   	   {
	   		   setTemp = 75;
	   	   }
	   else if(raw > 1228.5 && raw < 1638)
	   	   {
	   		   setTemp = 90;
	   	   }
	   else if(raw > 1638 && raw < 2047.5)
	   	   {
	   		   setTemp = 105;
	   	   }
	   else if(raw > 2047.5 && raw < 2457)
	   	   {
	   		   setTemp = 120;
	   	   }
	   else if(raw > 2457 && raw < 2866.5)
	   	   {
	   		   setTemp = 135;
	   	   }
	   else if(raw > 2866.5 && raw < 3276)
	   	   {
	   		   setTemp = 150;
	   	   }
	   else if(raw > 3276 && raw < 3685.5)
	   	   {
	   		   setTemp = 165;
	   	   }
	   else if(raw > 3685.5 && raw < 4095)
	   	   {
	   		   setTemp = 180;
	   	   }
	  //----------------------------------------------------------------------------------end user input----------------

	   //---------------------------------------------------------------------------------get tracking value-------------------------------
	   	   ADC_select_CH11(); //upper right LDR
	   	   HAL_ADC_Start(&hadc1);
	   	   HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	   	   ADCval[0] = HAL_ADC_GetValue(&hadc1);
	   	   HAL_ADC_Stop(&hadc1);

	   	   ADC_select_CH12(); //upper left LDR
	   	   HAL_ADC_Start(&hadc1);
	   	   HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	   	   ADCval[1] = HAL_ADC_GetValue(&hadc1);
	   	   HAL_ADC_Stop(&hadc1);

	   		ADC_select_CH13(); //lower right LDR
	   		HAL_ADC_Start(&hadc1);
	   		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	   		ADCval[2] = HAL_ADC_GetValue(&hadc1);
	   		HAL_ADC_Stop(&hadc1);

	   		ADC_select_CH0(); //lower left LDR
	   		HAL_ADC_Start(&hadc1);
	   		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	   		ADCval[3] = HAL_ADC_GetValue(&hadc1);
	   		HAL_ADC_Stop(&hadc1);

/*	   		if(ADCval[0] > 1000) //--------------------------- //upper right is in shadow therefore move down and/or left
	   		{
	   			if(ADCval[1] > 1000) //both upper LDRs in shadow therefore tilt down
	   			{
	   				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,25); //Second motor 25% voltage
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);   // Start pitch motor clock wise rotation
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); // Start pitch motor clock wise rotation
		   			HAL_Delay(500);
		   			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // Stop pitch motor clock wise rotation
	   			}

	   			else if(ADCVal[2] > 1000) //both right are in shadow therefore move left
	   			{
	   				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,25); //Second motor 25% voltage
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);   // Start azimuth motor clock wise rotation
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET); // Start azimuth motor clock wise rotation
		   			HAL_Delay(500);
		   			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); // Stop azimuth motor clock wise rotation
	   			}
	   		}

	   		if(ADCval[1] > 1000) //---------------------------------//upper left is in shadow therefore move down and/or right
	   		{
	   			if(ADCval[0] > 1000) //both upper LDRs in shadow therefore tilt down
	   			{
	   				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,25); //Second motor 25% voltage
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);   // Start pitch motor clock wise rotation
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); // Start pitch motor clock wise rotation
		   			HAL_Delay(500);
		   			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // Stop pitch motor clock wise rotation
	   			}

	   			else if(ADCVal[3] > 1000) //both left are in shadow therefore move left
	   			{
	   				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,25); //Second motor 25% voltage
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);   // Start azimuth motor clock wise rotation
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET); // Start azimuth motor clock wise rotation
			   			HAL_Delay(500);
			   			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); // Stop azimuth motor clock wise rotation
	   			}
	   		}

	   		if(ADCval[2] > 1000) //---------------------------------//lower right is in shadow therefore move up and/or left
	   		{
	   			if(ADCval[0] > 1000) //both right in shadow therefore move left
	   			{
	   				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,25); //Second motor 25% voltage
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);   // Start azimuth motor clock wise rotation
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET); // Start azimuth motor clock wise rotation
			   			HAL_Delay(500);
			   			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); // Stop azimuth motor clock wise rotation
	   			}

	   			else if(ADCVal[2] > 1000) //both lower are in shadow therefore move up
	   			{
	   				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,25); //Second motor 25% voltage
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);   // Start pitch motor clock wise rotation
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // Start pitch motor clock wise rotation
			   			HAL_Delay(500);
			   			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // Stop pitch motor clock wise rotation
	   			}
	   		}

	   		if(ADCval[3] > 1000) //---------------------------------//lower left is in shadow therefore move up and/or right
	   		{
	   			if(ADCval[1] > 1000) //both left in shadow therefore move right
	   			{
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);   // Start azimuth motor counter clock wise rotation
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); // Start azimuth motor counter clock wise rotation
		   			HAL_Delay(500);
		   			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);   // Stop azimuth motor counter clock wise rotation
	   			}

	   			else if(ADCVal[2] > 1000) //both lower are in shadow therefore move up
	   			{
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);   // Start pitch motor counter clock wise rotation
		   			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // Start pitch motor counter clock wise rotation
			   		HAL_Delay(500);
			   		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET); // Stop pitch motor counter clock wise rotation
	   			}
	   		}*/

	   //---------------------------------------------------------------------------------temperature control motor-----------------
	  /* if(avetemp = setTemp-3) //temperature reaches lower bound
	   {
 			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);   // Start temp motor CCW rotation
 			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET); // Start temp motor CCW rotation
 			 HAL_Delay(20000);
 			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);   // Stop temp motor CCW rotation
 			 counter = 1; //say that blind is halfway down
	   }

	   else if(avetemp = setTemp+3 && counter != 2) //temperature reaches upper bound
	   {
 			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);   // Start temp motor CCW rotation
 			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET); // Start temp motor CCW rotation
 			 HAL_Delay(20000);
 			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);   // Stop temp motor CCW rotation
 			 counter = 2; //say that blind is at the bottom
	   }
	   else if(avetemp < setTemp-3 && counter != 0)
	   {
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);   // Start temp motor CW rotation
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET); // Start temp motor CW rotation
			 HAL_Delay(40000);
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET); // Stop temp motor CW rotation
	   }*/
	   //----------------------------------------------------------------------------------end of temperature control motor--------------

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Prescaler = 19;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D1_Pin|D2_Pin|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11 
                          |D3_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA5 PA6 
                           PA7 PA8 PA9 PA10 
                           PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin PC9 PC11 */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB11 
                           D3_Pin PB13 PB14 PB15 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11 
                          |D3_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
