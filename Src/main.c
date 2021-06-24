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
#include "st7735.h" 
#include "fonts.h" 
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUF_LEN 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t r =  0 ;
volatile uint32_t adc_buf[BUF_LEN];
uint8_t tr_cmp = 0;
uint16_t adc_1 = 0;
uint16_t adc_2 = 0;
uint32_t time_count = 0;
float square = 0;
float v_min = 65535;
float v_max  = 0;
float mean = 0;
float v_rms = 0;
char rms_str[5] = "0";
char v_str[4] = "0";
bool is_drawing = 0;
float x_scale = 1;
float y_scale = 1;
uint8_t x_offset = 0;
uint8_t y_offset = 18;
bool trig = 0;
double freq = 0;
bool first_trig = 1;
bool adc_on = 0;
char hertz[5] = "0";
bool menu_mode = 0;
uint32_t freq_cnt = 0;
bool trig_type = 0;
bool trig_en  = 1;
uint16_t time1 = 0;
uint16_t time2 = 0;
uint8_t tim3_ovc = 0;
uint8_t menu_sel = 0;
uint8_t menu_lng = 1;
bool interpol = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void CalcVoltVal()
{
	square = 0;
	v_min = 65535;
	v_max  = 0;
    for (uint16_t i = 0; i < 160/x_scale; i++) 
		{
			square += pow((uint16_t)(adc_buf[i]), 2);
			if(v_min > (uint16_t)(adc_buf[i]))
			{
				v_min = (uint16_t)(adc_buf[i]);
			}
			else if(v_max < (uint16_t)(adc_buf[i]))
			{
				v_max = (uint16_t)(adc_buf[i]);
			}
    }
    mean = (square / ((float)(160/x_scale)));
    v_rms = (float)sqrt(mean)/1241 - 1.65;
		v_min = (float)v_min/1241 - 1.65;
		v_max = (float)v_max/1241 - 1.65;
	snprintf(rms_str, sizeof(rms_str), "%f", v_rms);
	ST7735_DrawString(25, 1, rms_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(1, 1, "rms", Font_7x10, ST7735_WHITE, ST7735_BLACK);
	snprintf(rms_str, sizeof(rms_str), "%f", v_min);
	ST7735_DrawString(25, 10, rms_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(1, 10, "min", Font_7x10, ST7735_WHITE, ST7735_BLACK);
	snprintf(rms_str, sizeof(rms_str), "%f", v_max);
	ST7735_DrawString(80, 10, rms_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(55, 10, "max", Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

void HzMeter()
{
	freq = time2-time1 + (tim3_ovc * 65536);
	freq = 1000*freq/64000000;
	freq = 1/freq;
	if((freq-1000) <= 0)
	{
		strcpy(hertz, "Hz");
	}
	else if((freq-1000)*(freq-1000000) <= 0)
	{
		freq = freq/1000;
		strcpy(hertz, "kHz");
	}
	else if((freq-1000000)*(freq-1000000000) <= 0)
	{
		freq = freq/1000000;
		strcpy(hertz, "MHz");
	}
	snprintf(v_str, sizeof(v_str), "%f", freq);
	ST7735_DrawString(55, 1, "f =", Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(80, 1, v_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(120, 1, hertz, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//TIM1->CR1 &= ~TIM_CR1_CEN;
	//time_count = TIM1->CNT;
	HAL_DMA_Abort(&hdma_adc1);
	adc_on = 0;
	tr_cmp = 1;
	trig_en = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin == GPIO_PIN_8)
	{
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		if(!menu_mode)
		{
			menu_mode = 1;
		}
		else
		{
			menu_mode = 0;
		}
		HAL_TIM_Base_Start_IT(&htim2);
	}
	if(GPIO_Pin == GPIO_PIN_9)
	{
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		if(!menu_mode)
		{
			x_scale = x_scale - 0.1;
		}
		else
		{
			if(menu_sel == 0)
			{
				if(trig_type == 1)
					trig_type = 0;
				else trig_type = 1;
			}
			if(menu_sel == 1)
			{
				if(interpol == 1)
					interpol = 0;
				else interpol = 1;
			}
		}
		HAL_TIM_Base_Start_IT(&htim2);
	}
	if(GPIO_Pin == GPIO_PIN_10)
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		if(!menu_mode)
		{
			y_scale = y_scale - 0.1;
		}
		else
		{
			if(menu_sel < menu_lng)
			{
			menu_sel = menu_sel + 1;
			}
			else menu_sel = 0;
		}
		HAL_TIM_Base_Start_IT(&htim2);
	}
	if(GPIO_Pin == GPIO_PIN_15)
	{
		if(!menu_mode)
		{
			HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
			y_scale = y_scale + 0.1;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		else
		{
			if(menu_sel >= menu_lng)
			{
			menu_sel = menu_sel - 1;
			}
			else menu_sel = menu_lng;
		}
	}
	if(GPIO_Pin == GPIO_PIN_3)
	{
		if(!menu_mode)
		{
			HAL_NVIC_DisableIRQ(EXTI3_IRQn);
			x_scale = x_scale + 0.1;
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM3)
	{		
	if(trig_en)
	{
	if(first_trig)
	{
		HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_buf, BUF_LEN);
		time1 = TIM3->CCR2;
		tim3_ovc = 0;
		first_trig = 0;
		trig = 1;
		ST7735_DrawString(1, 120, "TRIG", Font_7x10, ST7735_WHITE, ST7735_GREEN);
	}
	else if(!first_trig)
	{
		time2 = TIM3->CCR2;
	}	
	}
	}	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        if(htim->Instance == TIM2) 
        {
                HAL_TIM_Base_Stop_IT(&htim2); 
								__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
								__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
								__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
								__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
								__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
                NVIC_ClearPendingIRQ(EXTI9_5_IRQn); 
                HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
								NVIC_ClearPendingIRQ(EXTI15_10_IRQn); 
                HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); 
								NVIC_ClearPendingIRQ(EXTI3_IRQn); 
                HAL_NVIC_EnableIRQ(EXTI3_IRQn); 
        }
				if(htim->Instance == TIM3) 
				{
					tim3_ovc++;
				}	
}

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	ST7735_Init();
  ST7735_Backlight_On ();
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_SetRotation(3);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_DMA_Init(&hdma_adc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc1);
	HAL_TIM_Base_Init(&htim1);
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Init(&htim3);
	uint16_t x_pix = 0;
	uint16_t prev_y = 0;
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_buf, BUF_LEN);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(!menu_mode)
		{
		if(!trig)
		{
			ST7735_FillRectangle(0, 18, 160, 118, ST7735_BLACK);
			ST7735_DrawString(1, 120, "TRIG", Font_7x10, ST7735_WHITE, ST7735_RED);
		}
		if(tr_cmp == 1)
		{
		for(int i = 0; i<BUF_LEN; i++)
		{
			prev_y = adc_2;
			adc_1 = (uint16_t)(adc_buf[i]);
			adc_2 = (uint16_t)(adc_buf[i] >> 16);
			adc_1 = (int)adc_1/40;
			adc_2 = (int)adc_2/40;
			ST7735_DrawPixel(x_pix*x_scale + x_offset, (-adc_2+102)*y_scale + y_offset, ST7735_WHITE);
			ST7735_DrawPixel((x_pix+1)*x_scale + x_offset, (-adc_1+102)*y_scale + y_offset, ST7735_WHITE);
			if(interpol)
			{
				ST7735_DrawLine(x_pix*x_scale + x_offset, (-adc_2+102)*y_scale + y_offset, (x_pix+1)*x_scale + x_offset, (-adc_1+102)*y_scale + y_offset, ST7735_WHITE);
				ST7735_DrawLine(x_pix*x_scale + x_offset, (-adc_2+102)*y_scale + y_offset, (x_pix-1)*x_scale + x_offset, (-prev_y+102)*y_scale + y_offset, ST7735_WHITE);
			}
			//ST7735_DrawString(32, 2, "V", Font_7x10, ST7735_WHITE, ST7735_BLACK);
			//snprintf(t_str, sizeof(t_str), "%u", count);
			//ST7735_DrawString(50, 2, t_str, Font_7x10, ST7735_WHITE, ST7735_BLACK);
			x_pix = x_pix+2;
			if((float)(x_pix) >= (float)160/x_scale)
			{
				x_pix = 0;
				i = 0;
				ST7735_FillRectangle(0, 18, 160, 118, ST7735_BLACK);
				ST7735_FillRectangle(100, 1, 60, 9, ST7735_BLACK);
				//HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_buf, BUF_LEN);
				HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
				HzMeter();
				CalcVoltVal();
				//TIM1->CNT = 0;
				//TIM1->CR1 |= TIM_CR1_CEN;
				tr_cmp = 0;
				trig = 0;
				first_trig = 1;
				//freq = 0;
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
				NVIC_ClearPendingIRQ(EXTI15_10_IRQn);	
				HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
				trig_en = 1;
			}
		}
	}
}
		else
		{
			ST7735_DrawString(5, 0, "Trig type", Font_7x10, ST7735_WHITE, ST7735_BLACK);
			ST7735_DrawString(5, 20, "Interpolation", Font_7x10, ST7735_WHITE, ST7735_BLACK);
			switch(menu_sel){
				case 0:
					ST7735_DrawCircle(2, 0, 2, ST7735_YELLOW);
					if(trig_type == 0)
					{
						ST7735_DrawString(100, 0, " <Rise>", Font_7x10, ST7735_WHITE, ST7735_BLACK);
					}
					else
					{
						ST7735_DrawString(100, 0, " <Fall>", Font_7x10, ST7735_WHITE, ST7735_BLACK);
					}	
					break;
				case 1:
					ST7735_DrawCircle(2, 20, 2, ST7735_YELLOW);
					if(interpol == 0)
					{
						ST7735_DrawString(100, 20, " <Off>", Font_7x10, ST7735_WHITE, ST7735_BLACK);
					}
					else
					{
						ST7735_DrawString(100, 20, " <On>", Font_7x10, ST7735_WHITE, ST7735_BLACK);
					}
					break;
				}
				HAL_Delay(100);
				ST7735_FillScreen(ST7735_BLACK);		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INTERLFAST;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 15000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7735_RES_Pin|GPIO_PIN_13|ST7735_CS_Pin|ST7735_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7735_RES_Pin PB13 ST7735_CS_Pin ST7735_BL_Pin */
  GPIO_InitStruct.Pin = ST7735_RES_Pin|GPIO_PIN_13|ST7735_CS_Pin|ST7735_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
