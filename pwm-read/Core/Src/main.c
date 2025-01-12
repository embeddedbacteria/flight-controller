/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t IC_Val1_CH1 = 0, IC_Val2_CH1 = 0;
volatile uint32_t IC_Val1_CH2 = 0, IC_Val2_CH2 = 0;
volatile uint32_t IC_Val1_CH3 = 0, IC_Val2_CH3 = 0;
volatile uint32_t IC_Val1_CH4 = 0, IC_Val2_CH4 = 0;
volatile uint32_t IC_Val1_CH5 = 0, IC_Val2_CH5 = 0;
volatile uint32_t IC_Val1_CH6 = 0, IC_Val2_CH6 = 0;
volatile uint32_t IC_Val1_CH7 = 0, IC_Val2_CH7 = 0;
volatile uint32_t IC_Val1_CH8 = 0, IC_Val2_CH8 = 0;

volatile uint32_t Difference_CH1 = 0, Difference_CH2 = 0, Difference_CH3 = 0, Difference_CH4 = 0, Difference_CH5 = 0 , Difference_CH6 = 0, Difference_CH7 = 0, Difference_CH8 = 0;
uint8_t Is_First_Captured_CH1 = 0, Is_First_Captured_CH2 = 0;
uint8_t Is_First_Captured_CH3 = 0, Is_First_Captured_CH4 = 0;
uint8_t Is_First_Captured_CH5 = 0, Is_First_Captured_CH6 = 0;
uint8_t Is_First_Captured_CH7 = 0, Is_First_Captured_CH8 = 0;

volatile uint32_t Frequency_CH1 = 0, Frequency_CH2 = 0;
volatile uint32_t Frequency_CH3 = 0, Frequency_CH4 = 0;
volatile uint32_t Frequency_CH5 = 0, Frequency_CH6 = 0;
volatile uint32_t Frequency_CH7 = 0, Frequency_CH8 = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim ==&htim2)
	{

	/* Channel 1 */
	    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	    {
	        if (Is_First_Captured_CH1 == 0)
	        {
	            IC_Val1_CH1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            Is_First_Captured_CH1 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            IC_Val2_CH1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            __HAL_TIM_SET_COUNTER(htim, 0);

	            if (IC_Val2_CH1 > IC_Val1_CH1)
	            	{
	            		Difference_CH1 = IC_Val2_CH1 - IC_Val1_CH1;
	                 }
	            else
	                {
	                            Difference_CH1 = (0xFFFF - IC_Val1_CH1) + IC_Val2_CH1 + 1;
	                }
	            Frequency_CH1 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH1);
	            Is_First_Captured_CH1 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

	    /* Channel 2 */
	    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	    {
	        if (Is_First_Captured_CH2 == 0)
	        {
	            IC_Val1_CH2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	            Is_First_Captured_CH2 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            IC_Val2_CH2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	            __HAL_TIM_SET_COUNTER(htim, 0);

	            if (IC_Val2_CH2 > IC_Val1_CH2)
	            	            	{
	            	            		Difference_CH2 = IC_Val2_CH2 - IC_Val1_CH2;
	            	                 }
	            	            else
	            	                {
	            	                            Difference_CH2 = (0xFFFF - IC_Val1_CH2) + IC_Val2_CH2 + 1;
	            	                }
	            	            Frequency_CH2 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH2);
	            Is_First_Captured_CH2 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

	    /* Channel 3 */
	    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	    {
	        if (Is_First_Captured_CH3 == 0)
	        {
	            IC_Val1_CH3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
	            Is_First_Captured_CH3 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            IC_Val2_CH3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
	            __HAL_TIM_SET_COUNTER(htim, 0);

	            if (IC_Val2_CH3 > IC_Val1_CH3)
	            	            	{
	            	            		Difference_CH3 = IC_Val2_CH3 - IC_Val1_CH3;
	            	                 }
	            	            else
	            	                {
	            	                            Difference_CH3 = (0xFFFF - IC_Val1_CH3) + IC_Val2_CH3 + 1;
	            	                }
	            	            Frequency_CH3 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH3);
	            Is_First_Captured_CH3 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

	    /* Channel 4 */
	    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	    {
	        if (Is_First_Captured_CH4 == 0)
	        {
	            IC_Val1_CH4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
	            Is_First_Captured_CH4 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            IC_Val2_CH4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
	            __HAL_TIM_SET_COUNTER(htim, 0);

	            if (IC_Val2_CH4 > IC_Val1_CH4)
	            	            	{
	            	            		Difference_CH4 = IC_Val2_CH4 - IC_Val1_CH4;
	            	                 }
	            	            else
	            	                {
	            	                            Difference_CH4 = (0xFFFF - IC_Val1_CH4) + IC_Val2_CH4 + 1;
	            	                }
	            	            Frequency_CH4 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH4);
	            Is_First_Captured_CH4 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

}
	else if(htim ==&htim5)
		{

		/* Channel 1 */
		    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		    {
		        if (Is_First_Captured_CH5 == 0)
		        {
		            IC_Val1_CH5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		            Is_First_Captured_CH5 = 1;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		        }
		        else
		        {
		            IC_Val2_CH5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		            __HAL_TIM_SET_COUNTER(htim, 0);

		            if (IC_Val2_CH5 > IC_Val1_CH5)
		            	{
		            		Difference_CH5 = IC_Val2_CH5 - IC_Val1_CH5;
		                 }
		            else
		                {
		                            Difference_CH5 = (0xFFFF - IC_Val1_CH5) + IC_Val2_CH5 + 1;
		                }
		            Frequency_CH5 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH5);
		            Is_First_Captured_CH5 = 0;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		        }
		    }

		    /* Channel 2 */
		    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		    {
		        if (Is_First_Captured_CH6 == 0)
		        {
		            IC_Val1_CH6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		            Is_First_Captured_CH6 = 1;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		        }
		        else
		        {
		            IC_Val2_CH6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		            __HAL_TIM_SET_COUNTER(htim, 0);

		            if (IC_Val2_CH6 > IC_Val1_CH6)
		            	            	{
		            	            		Difference_CH6 = IC_Val2_CH6 - IC_Val1_CH6;
		            	                 }
		            	            else
		            	                {
		            	                            Difference_CH6 = (0xFFFF - IC_Val1_CH6) + IC_Val2_CH6 + 1;
		            	                }
		            	            Frequency_CH6 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH6);
		            Is_First_Captured_CH6 = 0;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
		        }

		    }

		    /* Channel 3 */

		    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		    {
		        if (Is_First_Captured_CH7 == 0)
		        {
		            IC_Val1_CH7 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		            Is_First_Captured_CH7 = 1;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		        }
		        else
		        {
		            IC_Val2_CH7 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		            __HAL_TIM_SET_COUNTER(htim, 0);

		            if (IC_Val2_CH7 > IC_Val1_CH7)
		            	            	{
		            	            		Difference_CH7 = IC_Val2_CH7 - IC_Val1_CH7;
		            	                 }
		            	            else
		            	                {
		            	                            Difference_CH7 = (0xFFFF - IC_Val1_CH7) + IC_Val2_CH7 + 1;
		            	                }
		            	            Frequency_CH7 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH7);
		            Is_First_Captured_CH7 = 0;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		        }
		    }

		    /* Channel 4 */
		    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		    {
		        if (Is_First_Captured_CH8 == 0)
		        {
		            IC_Val1_CH8 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		            Is_First_Captured_CH8 = 1;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		        }
		        else
		        {
		            IC_Val2_CH8 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		            __HAL_TIM_SET_COUNTER(htim, 0);

		            if (IC_Val2_CH8 > IC_Val1_CH8)
		            	            	{
		            	            		Difference_CH8 = IC_Val2_CH8 - IC_Val1_CH8;
		            	                 }
		            	            else
		            	                {
		            	                            Difference_CH8 = (0xFFFF - IC_Val1_CH8) + IC_Val2_CH8 + 1;
		            	                }
		            	            Frequency_CH8 = (HAL_RCC_GetPCLK1Freq()*2) / (42*Difference_CH8);
		            Is_First_Captured_CH8 = 0;
		            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
		        }
		    }

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */



  // TIM2 and TIM5 Interrupt Start
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);

  //TIM3 PWM START

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //TIM4 PWM START
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);


  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Difference_CH1);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Difference_CH2);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Difference_CH3);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,Difference_CH4);


	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Difference_CH5);
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,Difference_CH6);
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,Difference_CH7);
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,Difference_CH8);



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF-1;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19700;
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
  sConfigOC.Pulse = 0;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19700;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffff-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
