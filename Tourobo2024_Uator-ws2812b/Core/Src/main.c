/* USER CODE BEGIN Header */
/**LedUator用のプログラ?��?
 * 2024/09/03?��?新パンタ仕様に変更
 * 2024/09/05 パンタの縮伸,LEDのみを制御するように変更
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
#define LED_NUMBER 150
#define GREEN_LED_NUMBER 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
//for can
uint32_t id;
uint32_t dlc;
uint8_t data[8];
uint8_t failure = 0;

////for solenoid
//uint8_t solenoid[6] = {};

//fot panta
uint8_t panta_expand=0;
uint8_t panta_dunk=0;

//for LED
uint8_t LED_Data[3] = {};
uint32_t color;
uint32_t color_1;
uint32_t color_2;
uint32_t red = 0 << 16 | 140 << 8 | 0;
uint32_t orange = 9 << 16 | 140 << 8 | 0;
uint32_t yellow = 140 << 16 | 140 << 8 | 0;
uint32_t green = 140 << 16 | 0 << 8 | 0;
uint32_t light_blue =  140 << 16 | 0 << 8 | 140;
uint32_t blue = 0 << 16 | 0 << 8 | 140;
uint32_t purple = 0 << 16 | 140 << 8 | 140;
uint32_t white = 140 << 16 | 140 << 8 | 140;
uint32_t black = 0 << 16 | 0 << 8 | 0;
uint8_t bicolorData[24*LED_NUMBER];
uint8_t rainbowData[24*LED_NUMBER];
//for solenoid
GPIO_TypeDef* solv_ports[6] = {SOLV1_GPIO_Port, SOLV2_GPIO_Port, SOLV3_GPIO_Port, SOLV4_GPIO_Port, SOLV5_GPIO_Port, SOLV6_GPIO_Port};
uint16_t solv_pins[6] = {SOLV1_Pin, SOLV2_Pin, SOLV3_Pin, SOLV4_Pin, SOLV5_Pin, SOLV6_Pin};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void wait_290us(void);
void wait_50ms(void);
void ws2812b_send(void);
void ws2812b_send_green (void);
void ws2812b_bicolor(void);
void ws2812b_rainbow(void);
void rainbow_flow();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	if(hcan == &hcan2){
		CAN_RxHeaderTypeDef RxHeader;
		uint8_t RxData[2];
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {


			id = (RxHeader.IDE == CAN_ID_STD) ? RxHeader.StdId : RxHeader.ExtId;   // ID
			dlc = RxHeader.DLC;                                                   // DLC
			data[0] = RxData[0];                                                 // Data
			data[1] = RxData[1];

			panta_expand  =  data[0];//0 or 1

			if(panta_expand==0){
				HAL_GPIO_WritePin(solv_ports[0],solv_pins[0],GPIO_PIN_RESET);//panta off
			}else{
				HAL_GPIO_WritePin(solv_ports[0],solv_pins[0],GPIO_PIN_SET);//panta on
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
  MX_CAN2_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  //TIM
  HAL_TIM_Base_Start(&htim7);
  //CAN
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  CAN_FilterTypeDef filter;
  uint32_t fId   =  0x200 << 21;        // フィルターID
  uint32_t fMask = (0x7F0 << 21) | 0x4; // フィルターマスク

  filter.FilterIdHigh         = fId >> 16;             // フィルターIDの上�?16ビッ????��?��??��?��???��?��??��?��?
  filter.FilterIdLow          = fId;                   // フィルターIDの下�?16ビッ????��?��??��?��???��?��??��?��?
  filter.FilterMaskIdHigh     = fMask >> 16;           // フィルターマスクの上�?16ビッ????��?��??��?��???��?��??��?��?
  filter.FilterMaskIdLow      = fMask;                 // フィルターマスクの下�?16ビッ????��?��??��?��???��?��??��?��?
  filter.FilterScale          = CAN_FILTERSCALE_32BIT; // 32モー????��?��??��?��???��?��??��?��?
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;      // FIFO0へ格????��?��??��?��???��?��??��?��?
  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK; // IDマスクモー????��?��??��?��???��?��??��?��?
  filter.SlaveStartFilterBank = 0;
  filter.FilterActivation     = ENABLE;

  HAL_CAN_ConfigFilter(&hcan2, &filter);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ws2812b_send_green ();

	  for(int i=0; i<10; i++){
		switch(data[1]){

		case 0:
			color_1 = red;
			color_2 = black;
			ws2812b_bicolor();
			break;

		case 1:
			color_1 = orange;
			color_2 = black;
			ws2812b_bicolor();
			break;

		case 2:
			color_1 = yellow;
			color_2 = black;
			ws2812b_bicolor();
			break;

		case 3:
			color = green;
			color_2 = black;
			ws2812b_bicolor();
			break;

		case 4:
			color_1 = light_blue;
			color_2 = black;
			ws2812b_bicolor();
			break;

		case 5:
			color_1 = red;
			color_2 = blue;
			ws2812b_bicolor();
			break;

		case 6:
			color_1 = orange;
			color_2 = light_blue;
			ws2812b_bicolor();
			break;

		case 7:
			color_1 = blue;
			color_2 = black;
			ws2812b_bicolor();
			break;

		case 8:
			color_1 = purple;
			color_2 = black;
			ws2812b_bicolor();
			break;

		case 9:
			color_1 = white;
			color_2 = black;
			ws2812b_bicolor();
			break;

		case 10:
			ws2812b_rainbow();
			break;

		case 11:
			rainbow_flow();
			break;

		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SOLV6_Pin|SOLV5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SOLV4_Pin|SOLV3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SOLV2_Pin|SOLV1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC2A_Pin ENC3A_Pin ENC3B_Pin */
  GPIO_InitStruct.Pin = ENC2A_Pin|ENC3A_Pin|ENC3B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC4A_Pin */
  GPIO_InitStruct.Pin = ENC4A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC4A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SOLV6_Pin SOLV5_Pin */
  GPIO_InitStruct.Pin = SOLV6_Pin|SOLV5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SOLV4_Pin SOLV3_Pin */
  GPIO_InitStruct.Pin = SOLV4_Pin|SOLV3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SOLV2_Pin SOLV1_Pin */
  GPIO_InitStruct.Pin = SOLV2_Pin|SOLV1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1A_Pin ENC1B_Pin */
  GPIO_InitStruct.Pin = ENC1A_Pin|ENC1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void wait_290us(){
	htim7.Instance->CNT = 0;
	while((htim7.Instance->CNT)<290);
}

void wait_50ms(){
	for(int i=0; i<50; i++){
		htim7.Instance->CNT = 0;
		while((htim7.Instance->CNT)<999);
	}
}

void ws2812b_send ()
{
	uint8_t sendData[24*LED_NUMBER];

	for(int i=0; i<LED_NUMBER; i++){
		for(int j=0; j<24; j++){
			if (((color>>(23-j))&0x01) == 1) {
				sendData[i*24 + j] = 0b11111000;  // store 1
			}
			else{
				sendData[i*24 + j] = 0b11000000;  // store 0
			}
		}
	}

	HAL_SPI_Transmit(&hspi2, sendData, 24*LED_NUMBER, 1000);
	wait_290us();
}

void ws2812b_send_green ()
{
	uint32_t color = 153<<16 | 0<<8 | 0;
	uint8_t sendData[24*LED_NUMBER];

	for(int i=0; i<LED_NUMBER; i++){
		for(int j=0; j<24; j++){
			if (((color>>(23-j))&0x01) == 1) {
				sendData[i*24 + j] = 0b11111000;  // store 1
			}
			else{
				sendData[i*24 + j] = 0b11000000;  // store 0
			}
		}
	}

	HAL_SPI_Transmit(&hspi1, sendData, 24*LED_NUMBER, 1000);
	wait_290us();
}

void ws2812b_bicolor(){

	for(int i=0; i<LED_NUMBER; i++){
		for(int j=0; j<24; j++){
			int remainder = i%2;
			switch(remainder){
			case 0:
				if (((color_1>>(23-j))&0x01) == 1) {
					bicolorData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					bicolorData[i*24 + j] = 0b11000000;  // store 0
				}
				break;
			case 1:
				if (((color_2>>(23-j))&0x01) == 1) {
					bicolorData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					bicolorData[i*24 + j] = 0b11000000;  // store 0
				}
				break;
			}
		}
	}

	HAL_SPI_Transmit(&hspi2, bicolorData, 24*LED_NUMBER, 1000);
	wait_290us();
}

void ws2812b_rainbow(){
	for(int i=0; i<LED_NUMBER; i++){
		for(int j=0; j<24; j++){
			int remainder = i%7;

			switch(remainder){

			case 0:
				if (((red>>(23-j))&0x01) == 1) {
					rainbowData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					rainbowData[i*24 + j] = 0b11000000;  // store 0
				}
				break;

			case 1:
				if (((orange>>(23-j))&0x01) == 1) {
					rainbowData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					rainbowData[i*24 + j] = 0b11000000;  // store 0
				}
				break;

			case 2:
				if (((yellow>>(23-j))&0x01) == 1) {
					rainbowData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					rainbowData[i*24 + j] = 0b11000000;  // store 0
				}
				break;

			case 3:
				if (((green>>(23-j))&0x01) == 1) {
					rainbowData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					rainbowData[i*24 + j] = 0b11000000;  // store 0
				}
				break;

			case 4:
				if (((light_blue>>(23-j))&0x01) == 1) {
					rainbowData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					rainbowData[i*24 + j] = 0b11000000;  // store 0
				}
				break;

			case 5:
				if (((blue>>(23-j))&0x01) == 1) {
					rainbowData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					rainbowData[i*24 + j] = 0b11000000;  // store 0
				}
				break;

			case 6:
				if (((purple>>(23-j))&0x01) == 1) {
					rainbowData[i*24 + j] = 0b11111000;  // store 1
				}
				else{
					rainbowData[i*24 + j] = 0b11000000;  // store 0
				}
				break;
			}
		}
	}

	HAL_SPI_Transmit(&hspi2, rainbowData, 24*LED_NUMBER, 1000);
	wait_290us();
}

void rainbow_flow(){

	uint8_t hold[24] = {0};

	for(int i=0; i<24; i++){
		hold[i] = rainbowData[i];
	}

	for(int i=0; i<24*(LED_NUMBER-1); i++){
		rainbowData[i] = rainbowData[i+24];
	}

	for(int i=0; i<24; i++){
		rainbowData[24*(LED_NUMBER-1)+i] = hold[i];
	}

	HAL_SPI_Transmit(&hspi2, rainbowData, 24*LED_NUMBER, 1000);
	wait_50ms();

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
