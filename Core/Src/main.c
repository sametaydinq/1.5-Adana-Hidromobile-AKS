/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t adcbuffer[6];

uint32_t Eys_Voltage;
uint32_t Aks_Voltage;
uint32_t Smps_Voltage;
uint32_t Bms_Voltage;
uint32_t Engine_Driver_Voltage;
uint32_t Gaz_Sensor;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   if(hadc->Instance == ADC1)
   {
	   Eys_Voltage = adcbuffer[1];
	   Aks_Voltage = adcbuffer[3];
	   Smps_Voltage = adcbuffer[4];
	   Bms_Voltage = adcbuffer[5];
	   Engine_Driver_Voltage = adcbuffer[2];
	   Gaz_Sensor = adcbuffer[0];
   }

}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxTransmitter;   //can adı
uint8_t voltage_result[5];   //can içerisindeki data boyutu
uint32_t TxMailBox;       //can kutusu

uint32_t adc_result[10];   //adc değerleri için kutu,
									//sensor değerleri içinde kutu açılacak



CAN_RxHeaderTypeDef Receiver;
uint8_t Receiver_data[8];
uint32_t sender_info[5];





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
  MX_CAN_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1,adcbuffer,6);
  HAL_CAN_Start(&hcan);






  //araç açılışta sürüş modunda olacak.
  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);


  /*
   Sürüş moduna geçmek için yapılması gerekenler:
   BMS,Motor Sürücü,Telemetri açılacak. SMPS kapanacak.
   Ana kontaktör kontrolünde P channel kullanılır. Lojik 0 verilirse, mosfet akım geçirir. Tesisatta kontaktöre 12V gider ve, kontaktör akım geçirir.
   SSR röle ve SMPS Batarya kontaktörünü kapatmak için Lojik 1 verilir.
   Motor sürücü kontaktöründen akım geçmesi için Lojik 0 verilmeli.
  */
  HAL_GPIO_WritePin(SIGN_BMS_GPIO_Port,SIGN_BMS_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır
  HAL_GPIO_WritePin(RELAY_ENGINE_12V_GPIO_Port,RELAY_ENGINE_12V_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır
  HAL_GPIO_WritePin(SIGN_Telemetry_GPIO_Port,SIGN_Telemetry_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır---
  HAL_GPIO_WritePin(SIGN_EYS_GPIO_Port,SIGN_EYS_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(SIGN_SMPS_GPIO_Port,SIGN_SMPS_Pin, GPIO_PIN_SET);//Lojik 1 açılır, Lojik 0 kapanır
  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,RELAY_MAIN_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
  HAL_GPIO_WritePin(RELAY_AC_IN_GPIO_Port,RELAY_AC_IN_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
  HAL_GPIO_WritePin(RELAY_SMPS_BAT_GPIO_Port,RELAY_SMPS_BAT_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
  HAL_Delay(1000);
  HAL_GPIO_WritePin(RELAY_ENGINE_96V_GPIO_Port,RELAY_ENGINE_96V_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır






  TxTransmitter.StdId = 0x023;
  TxTransmitter.DLC = 8;
  TxTransmitter.IDE = CAN_ID_STD;



  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*
	   Eys_Voltage = adcbuffer[1];
	   Aks_Voltage = adcbuffer[3];
	   Smps_Voltage = adcbuffer[4];
	   Bms_Voltage = adcbuffer[5];
	   Engine_Driver_Voltage = adcbuffer[2];
	   Gaz_Sensor = adcbuffer[0];

	   */
    /* USER CODE END WHILE */
	  HAL_ADC_ConvCpltCallback(&hadc1);



	  /*
	  if(HAL_GPIO_ReadPin(IN_AC_DETECT_GPIO_Port,IN_AC_DETECT_Pin) == GPIO_PIN_RESET){
		  //BURASI ARAÇ sürüş MODU DURUMU


		  HAL_GPIO_WritePin(RELAY_SMPS_BAT_GPIO_Port,RELAY_SMPS_BAT_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
		  HAL_Delay(20);
		  HAL_GPIO_WritePin(RELAY_AC_IN_GPIO_Port,RELAY_AC_IN_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(SIGN_SMPS_GPIO_Port,SIGN_SMPS_Pin, GPIO_PIN_SET);//Lojik 1 açılır, Lojik 0 kapanır
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(RELAY_ENGINE_12V_GPIO_Port,RELAY_ENGINE_12V_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(RELAY_ENGINE_96V_GPIO_Port,RELAY_ENGINE_96V_Pin, GPIO_PIN_SET);//Lojik 1 akım geçirmez, Lojik 0 akım geçirir
  		  HAL_Delay(20);
		  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
	  	  }

	  else{
		  //burası şarj modu

		  HAL_GPIO_WritePin(RELAY_ENGINE_12V_GPIO_Port,RELAY_ENGINE_12V_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
		  HAL_GPIO_WritePin(RELAY_ENGINE_96V_GPIO_Port,RELAY_ENGINE_96V_Pin, GPIO_PIN_RESET);//Lojik 1 akım geçirmez, Lojik 0 akım geçirir
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(RELAY_SMPS_BAT_GPIO_Port,RELAY_SMPS_BAT_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(RELAY_AC_IN_GPIO_Port,RELAY_AC_IN_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(SIGN_SMPS_GPIO_Port,SIGN_SMPS_Pin, GPIO_PIN_RESET);//Lojik 1 açılır, Lojik 0 kapanır


	  }

	  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
	  */


	  static bool prev_ac_status = false;
	  	  const bool ac_status = HAL_GPIO_ReadPin(IN_AC_DETECT_GPIO_Port,IN_AC_DETECT_Pin) == GPIO_PIN_SET;

	  	  if (ac_status != prev_ac_status) {  //eğer önceki durum ve şimdiki durum farklı olursa işleme girer
	  		                                  //işlem seçeneği ise mevcut ac değerinin ne olduğuna göre yapılır
	  		  prev_ac_status = ac_status;
	  		  if(ac_status){
				  //şarj modu
	  			  HAL_GPIO_WritePin(RELAY_ENGINE_96V_GPIO_Port,RELAY_ENGINE_96V_Pin, GPIO_PIN_RESET);//Lojik 1 akım geçirmez, Lojik 0 akım geçirir
	  			  HAL_Delay(500);
	  			  HAL_GPIO_WritePin(SIGN_EYS_GPIO_Port,SIGN_EYS_Pin,GPIO_PIN_RESET);
	  			  HAL_Delay(500);
	  			  HAL_GPIO_WritePin(RELAY_ENGINE_12V_GPIO_Port,RELAY_ENGINE_12V_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
		  	  	  HAL_Delay(1000);
		  	  	  HAL_GPIO_WritePin(RELAY_SMPS_BAT_GPIO_Port,RELAY_SMPS_BAT_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır
		  	  	  HAL_Delay(1000);
		  	  	  HAL_GPIO_WritePin(RELAY_AC_IN_GPIO_Port,RELAY_AC_IN_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır
		  	  	  HAL_Delay(1000);
		  	  	  HAL_GPIO_WritePin(SIGN_SMPS_GPIO_Port,SIGN_SMPS_Pin, GPIO_PIN_RESET);//Lojik 1 açılır, Lojik 0 kapanır
	  			  }

	  		  else{
	  		  	  //sürüş modu
	  			  HAL_GPIO_WritePin(RELAY_SMPS_BAT_GPIO_Port,RELAY_SMPS_BAT_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
		  	  	  HAL_Delay(20);
		  	  	  HAL_GPIO_WritePin(RELAY_AC_IN_GPIO_Port,RELAY_AC_IN_Pin, GPIO_PIN_RESET);//Lojik 1 kapanır, Lojik 0 açılır
		  	  	  HAL_Delay(1000);
		  	  	  HAL_GPIO_WritePin(SIGN_SMPS_GPIO_Port,SIGN_SMPS_Pin, GPIO_PIN_SET);//Lojik 1 açılır, Lojik 0 kapanır
		  	  	  HAL_Delay(1000);
		  	  	  HAL_GPIO_WritePin(RELAY_ENGINE_12V_GPIO_Port,RELAY_ENGINE_12V_Pin, GPIO_PIN_SET);//Lojik 1 kapanır, Lojik 0 açılır
		  	  	  HAL_Delay(1000);
		  	  	  HAL_GPIO_WritePin(SIGN_EYS_GPIO_Port,SIGN_EYS_Pin,GPIO_PIN_SET);
		  	  	  HAL_Delay(1000);
		  	  	  HAL_GPIO_WritePin(RELAY_ENGINE_96V_GPIO_Port,RELAY_ENGINE_96V_Pin, GPIO_PIN_SET);//Lojik 1 akım geçirmez, Lojik 0 akım geçirir
  		  	  	  HAL_Delay(20);
		  	  	  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);

	  		  }
	  	  }






	  voltage_result[0] = Eys_Voltage;
	  voltage_result[1] =Aks_Voltage;
	  voltage_result[2] =Smps_Voltage;
	  voltage_result[3] =Bms_Voltage;
	  voltage_result[4] =Engine_Driver_Voltage;

	  voltage_result[5] = 31;
	  voltage_result[6] = 52;
	  voltage_result[7] = 69;


	  HAL_Delay(50);

	  HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, voltage_result , &TxMailBox);





	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &Receiver, Receiver_data);

	  sender_info[0] = Receiver.StdId;
	  sender_info[1] = Receiver.ExtId;
	  sender_info[2] = Receiver.IDE;
	  sender_info[3] = Receiver.RTR;
      sender_info[4] = Receiver.DLC;

      /*
      if(Receiver_data[0] == 54)
      HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, SET);
      else
          HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, RESET);
	  */

      /*
      if(Receiver_data[1] > 55)
      HAL_GPIO_WritePin(SELENOID_VALF_GPIO_Port, SELENOID_VALF_Pin, SET);
      else
          HAL_GPIO_WritePin(SELENOID_VALF_GPIO_Port, SELENOID_VALF_Pin, RESET);


      if(Receiver_data[2] == 65)
    		 HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, SET);
      else
		 HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, RESET);

	*/


/*
	  if(Eys_Voltage <10)          //eğer voltaj yoksa led yaktık
		  HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
	  if(Aks_Voltage <10)
	  		  HAL_GPIO_WritePin(SELENOID_VALF_GPIO_Port, SELENOID_VALF_Pin, GPIO_PIN_SET);

	  if(Smps_Voltage >3500){
	  		  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_SET);
	  		HAL_Delay(1000);
	  }
	  */
	  //read sürücü 12v

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef can_filter_type;

  can_filter_type.FilterBank = 13;
  can_filter_type.FilterIdHigh = 0;
		  can_filter_type.FilterIdLow = 0;
		  can_filter_type.FilterMaskIdHigh = 0;
		  can_filter_type.FilterMaskIdLow = 0;
		  can_filter_type.FilterFIFOAssignment = CAN_RX_FIFO0;
		  can_filter_type.FilterBank = 0;
		  can_filter_type.FilterMode = CAN_FILTERMODE_IDMASK;
		  can_filter_type.FilterScale = CAN_FILTERSCALE_32BIT;
		  can_filter_type.FilterActivation = ENABLE;
		  can_filter_type.SlaveStartFilterBank = 14;

		  HAL_CAN_ConfigFilter(&hcan, &can_filter_type);

  /* USER CODE END CAN_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIGN_EYS_Pin|RELAY_ENGINE_12V_Pin|SIGN_Telemetry_Pin|SIGN_BMS_Pin
                          |SIGN_SMPS_Pin|RELAY_SMPS_BAT_Pin|RELAY_AC_IN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_ENGINE_96V_Pin|RELAY_MAIN_Pin|SELENOID_VALF_Pin|LIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SIGN_EYS_Pin RELAY_ENGINE_12V_Pin SIGN_Telemetry_Pin SIGN_BMS_Pin
                           SIGN_SMPS_Pin RELAY_SMPS_BAT_Pin RELAY_AC_IN_Pin */
  GPIO_InitStruct.Pin = SIGN_EYS_Pin|RELAY_ENGINE_12V_Pin|SIGN_Telemetry_Pin|SIGN_BMS_Pin
                          |SIGN_SMPS_Pin|RELAY_SMPS_BAT_Pin|RELAY_AC_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_AC_DETECT_Pin */
  GPIO_InitStruct.Pin = IN_AC_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_AC_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_ENGINE_96V_Pin RELAY_MAIN_Pin SELENOID_VALF_Pin LIGHT_Pin */
  GPIO_InitStruct.Pin = RELAY_ENGINE_96V_Pin|RELAY_MAIN_Pin|SELENOID_VALF_Pin|LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
