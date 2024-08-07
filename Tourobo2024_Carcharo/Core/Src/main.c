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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_math.h"
#include "sockets.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DELTA_T 0.01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_CAN3_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//UDP
#define F7_ADDR "192.168.21.111"
#define PC_ADDR "192.168.21.100"
#define F7_PORT 4001
#define PC_PORT 4001

// ???¿½?¿½??¿½?¿½?ã‚¹ãƒˆç”¨
uint32_t id;
uint32_t dlc;
uint32_t data[8];
//int16_t omega;
int16_t torque;

typedef struct {
	// Input torque target
	int16_t TargetTorque;
	// Current angle
	int16_t Angle;
	// Current angular velocity
	int16_t AngularVelocity;
	// Input torque feedback
	int16_t FeedbackTorque;
	// Motor Temperature
	uint8_t MotorTemperature;
	// Update Check
	uint32_t Event;
} RobomasterTypedef;

void Robomaster_InitZero(RobomasterTypedef *Robomaster) {
	// Input torque target
	Robomaster->TargetTorque = 0;
	// Current angle
	Robomaster->Angle = 0;
	// Current angular velocity
	Robomaster->AngularVelocity = 0;
	// Input torque feedback
	Robomaster->FeedbackTorque = 0;
	// Motor Temperature
	Robomaster->MotorTemperature = 0;
	// Update Check
	Robomaster->Event = 0;
}

void Robomaster_RxCAN(RobomasterTypedef *Robomaster, uint8_t *RxData) {
	// Current angle
	Robomaster->Angle = RxData[0] >> 8 | RxData[1];
	// Current angular velocity
	Robomaster->AngularVelocity = RxData[2] >> 8 | RxData[3];
	// Input torque feedback
	Robomaster->FeedbackTorque = RxData[4] >> 8 | RxData[5];
	// Motor temperature
	Robomaster->MotorTemperature = RxData[6];
	// Update Check
	Robomaster->Event = 1;
}

RobomasterTypedef Robomaster[4];

// CANå—ä¿¡ã‚³ãƒ¼ãƒ«ãƒãƒƒã‚¯é–¢æ•°
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan2) {
		CAN_RxHeaderTypeDef RxHeader;
		uint8_t RxData[8];
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			id = RxHeader.StdId - 0x200;
			dlc = RxHeader.DLC;
			for (size_t i = 0; i < 8; i++) {
				data[i] = RxData[i];
			}

			Robomaster[id - 1].AngularVelocity = data[2] << 8 | data[3];

			Robomaster_RxCAN(&Robomaster[id - 1], &RxData[0]);

			// é€ä¿¡
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) {
				// é€ä¿¡ç”¨æ§‹ï¿½??ä½“ï¿½????¿½?¿½??¿½?¿½å®šç¾©
				CAN_TxHeaderTypeDef TxHeader;
				// IDã®è¨­???¿½?¿½??¿½?¿½?
				TxHeader.StdId = 0x200;
				// æ¨™æº–IDã‚’ä½¿ç”¨
				TxHeader.IDE = CAN_ID_STD;
				// ???¿½?¿½??¿½?¿½?ãƒ¼ã‚¿ãƒ•ãƒ¬ãƒ¼???¿½?¿½??¿½?¿½? or ãƒªãƒ¢ãƒ¼ãƒˆãƒ•ãƒ¬ãƒ¼???¿½?¿½??¿½?¿½?
				TxHeader.RTR = CAN_RTR_DATA;
				// ???¿½?¿½??¿½?¿½?ãƒ¼ã‚¿é•·???¿½?¿½??¿½?¿½? [byte]
				TxHeader.DLC = 8;
				// ã‚¿ã‚¤???¿½?¿½??¿½?¿½?ã‚¹ã‚¿ãƒ³???¿½?¿½??¿½?¿½?
				TxHeader.TransmitGlobalTime = DISABLE;
				// 8byteã®é€ä¿¡???¿½?¿½??¿½?¿½?ãƒ¼ã‚¿
				uint8_t TxData[8] = { 0 };
				for (int i = 0; i < 4; i++) {
					TxData[2 * i] = Robomaster[i].TargetTorque >> 8;
					TxData[2 * i + 1] = Robomaster[i].TargetTorque & 0x00FF;
				}
				// é€ä¿¡ã«ä½¿ã£ãŸTxMailboxãŒï¿½????¿½?¿½??¿½?¿½ç´ã•ã‚Œã‚‹
				uint32_t TxMailbox;
				// ãƒ¡???¿½?¿½??¿½?¿½?ã‚»ãƒ¼ã‚¸é€ä¿¡
				HAL_CAN_AddTxMessage(&hcan2, &TxHeader, &TxData, &TxMailbox);
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
  MX_CAN3_Init();
  /* USER CODE BEGIN 2 */

	/* ãƒ•ã‚£ãƒ«ã‚¿è¨­????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½? */
	/* ãƒã‚¹ã‚¯ãƒ¢ãƒ¼ãƒ‰ã§ã¯fid & fmask == receve_id & fmaskãªã‚‰FIFOã«å—ä¿¡????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?ãƒ¼ã‚¿ã‚’ï¿½?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½ç´ï¼Œãã‚Œä»¥å¤–ï¿½?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½ç ´????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?ã—ã¾ã™ï¿½?
	 * æ¨™æº–IDã®å ´åˆï¼ŒfidãŒï¿½?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½ç´ã•ã‚Œã‚‹ã®ã¯ãƒ¬ã‚¸ã‚¹ã‚¿ã®ä¸Šã‹????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?11bitã§ã™ï¿½?
	 * ã¤ã¾ã‚Šï¼Œæ¨™æº–IDã‚’ä½¿????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?å ´åˆï¼Œfidã‚’å·¦5bitã‚·ãƒ•ãƒˆã•ã›ã¦HIGHã«æ›¸ãè¾¼ã¿?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½LOWã«ã¯0ã‚’æ›¸ãè¾¼???¿½?¿½??¿½?¿½????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?ã“ã¨ã«ãªã‚Šã¾ã™ï¿½?
	 */
	// ãƒ•ã‚£ãƒ«ã‚¿ã®æ§‹ï¿½??ä½“å®šç¾©
	CAN_FilterTypeDef filter_hcan2;
	// IDã¨ãƒã‚¹ã‚¯ã®è¨­å®šï¼ˆã“ã®????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?ã¿åˆã‚ã›ï¿½?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½å ´åˆï¿½?0x200 -- 0x20Fã¾ã§ã‚’å–å¾—ã§ãã‚‹?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?
	uint32_t fid = 0x200;
	uint32_t fmask = 0xFF0;
	/* ???¿½?¿½??¿½?¿½?ã‚¹ãƒˆç”¨ */
//	fid = 0;
//	fmask = 0;
	/**/
	filter_hcan2.FilterIdHigh = fid << 5;
	filter_hcan2.FilterIdLow = 0;
	filter_hcan2.FilterMaskIdHigh = fmask << 5;
	filter_hcan2.FilterMaskIdLow = 0;
	// "filter_hcan2"ã«é€šã—ãŸå¾Œï¿½?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?ãƒ¼ã‚¿ã®æ ¼ç´ï¿½?????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½ã«"FIFO0"ã‚’é¸????¿½?¿½??¿½?¿½???¿½?¿½??¿½?¿½?
	filter_hcan2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter_hcan2.FilterBank = 0;
	filter_hcan2.FilterMode = CAN_FILTERMODE_IDMASK;
	filter_hcan2.FilterScale = CAN_FILTERSCALE_32BIT;
	filter_hcan2.FilterActivation = CAN_FILTER_ENABLE;
	filter_hcan2.SlaveStartFilterBank = 0;
	// filteré©ç”¨
	HAL_CAN_ConfigFilter(&hcan2, &filter_hcan2);
	// hcan2é–‹ï¿½?
	HAL_CAN_Start(&hcan2);
	// hcan2å‰²ã‚Šè¾¼ã¿æœ‰åŠ¹???¿½?¿½??¿½?¿½?
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

	// ãƒ­ãƒœï¿½???¿½?¿½ã‚¹æ§‹ï¿½??ä½“ï¿½???¿½?¿½æœŸï¿½?
	for (int i = 0; i < 4; i++) {
		Robomaster_InitZero(&Robomaster[i]);
	}

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  * @brief CAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 6;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF1 PF2 PF3 PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG7 PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
	int16_t rxbuf[16] = { 0 };
	int16_t txbuf[20] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
			17, 18, 19, 20 };
	float32_t Kp = 10;
	float32_t Ki = 0.01;
	float32_t difference = 0;
	float32_t pre_difference = 0;
	int16_t TagetAngularVelocity[4] = { 0 };
	float32_t p_value;
	float32_t i_value;
	//ã‚¢ãƒ‰ãƒ¬ã‚¹ã‚’å®£?¿½?
	struct sockaddr_in rxAddr, txAddr;
	//ã‚½ã‚±?¿½?ãƒˆã‚’ä½œï¿½??
	int socket = lwip_socket(AF_INET, SOCK_DGRAM, 0);
	//ã‚¢ãƒ‰ãƒ¬ã‚¹ã®ãƒ¡ãƒ¢ãƒªã‚’ç¢º?¿½?
	memset((char*) &txAddr, 0, sizeof(txAddr));
	memset((char*) &rxAddr, 0, sizeof(rxAddr));
	//ã‚¢ãƒ‰ãƒ¬ã‚¹ã®æ§‹ï¿½??ä½“ï¿½??¿½?¿½?ãƒ¼ã‚¿ã‚’å®šç¾©
	rxAddr.sin_family = AF_INET; //ãƒ—ãƒ­ãƒˆã‚³ãƒ«ãƒ•ã‚¡ãƒŸãƒªã®è¨­?¿½?(IPv4ã«è¨­?¿½?)
	rxAddr.sin_len = sizeof(rxAddr); //ã‚¢ãƒ‰ãƒ¬ã‚¹ã®?¿½?ãƒ¼ã‚¿ã‚µã‚¤ã‚º
	rxAddr.sin_addr.s_addr = INADDR_ANY; //ã‚¢ãƒ‰ãƒ¬ã‚¹ã®è¨­?¿½?(ä»Šå›ã¯ã™ã¹ã¦ã®ã‚¢ãƒ‰ãƒ¬ã‚¹ã‚’å—ã‘ï¿½??¿½ã‚Œã‚‹ãŸã‚INADDR_ANY)
	rxAddr.sin_port = lwip_htons(PC_PORT); //ãƒï¿½??¿½ãƒˆï¿½??¿½?¿½??¿½?
	txAddr.sin_family = AF_INET; //ãƒ—ãƒ­ãƒˆã‚³ãƒ«ãƒ•ã‚¡ãƒŸãƒªã®?¿½??¿½?(IPv4ã«è¨­?¿½?)
	txAddr.sin_len = sizeof(txAddr); //ã‚¢ãƒ‰ãƒ¬ã‚¹ã®?¿½?ãƒ¼ã‚¿ã®ã‚µã‚¤ã‚º
	txAddr.sin_addr.s_addr = inet_addr(PC_ADDR); //ã‚¢ãƒ‰ãƒ¬ã‚¹ã®è¨­?¿½?
	txAddr.sin_port = lwip_htons(PC_PORT); //ãƒï¿½??¿½ãƒˆï¿½??¿½?¿½??¿½?
	// whileã§bindã‚’å¾?ã£ã¦ã¿ã‚‹ã¨ã?ã¾ãè¡Œãå¯èƒ½æ€§ã‚?
//	(void) lwip_bind(socket, (struct sockaddr*) &rxAddr, sizeof(rxAddr)); //IPã‚¢ãƒ‰ãƒ¬ã‚¹ã¨ã‚½ã‚±?¿½?ãƒˆã‚’ç´ä»˜ã‘ã¦å—ä¿¡ã‚’ã§ãã‚‹çŠ¶æ…‹ã«
	while(lwip_bind(socket, (struct sockaddr*) &rxAddr, sizeof(rxAddr)) < 0) {

	}
	socklen_t n; //å—ä¿¡ã—ãŸ?¿½?ãƒ¼ã‚¿ã®ã‚µã‚¤ã‚º
	socklen_t len = sizeof(rxAddr); //rxAddrã®ã‚µã‚¤ã‚º

	/* Infinite loop */
	for (;;) {
		lwip_sendto(socket, (uint8_t*) txbuf, sizeof(txbuf), 0,
				(struct sockaddr*) &txAddr, sizeof(txAddr)); //å—ä¿¡ã—ãŸã‚‰ï¿½??¿½ä¿¡ã™ã‚‹
		n = lwip_recvfrom(socket, (uint8_t*) rxbuf, sizeof(rxbuf), (int) NULL,
				(struct sockaddr*) &rxAddr, &len); //å—ä¿¡å‡¦?¿½?(blocking)

//		int16_t test = Robomaster[0].Angle;z
		//ãƒ¢ãƒ¼ã‚¿ãƒ¼ã®é€Ÿåº¦åˆ¶å¾¡
//		for (int i = 0; i < 4; i++) {
//			if (Robomaster[i].Event == 1) {
//				Robomaster[i].Event = 0;
//
//				TagetAngularVelocity[i] = rxbuf[i];
//
//				difference = TagetAngularVelocity[i] - Robomaster[i].AngularVelocity;
//
//				//piåˆ¶å¾¡
//				p_value = difference;
//				i_value += (difference + pre_difference) * (DELTA_T / 2);
//				Robomaster[i].TargetTorque = p_value * Kp; //+ i_value*Ki;
//				pre_difference = difference;
//
//				txbuf[i] = Robomaster[i].AngularVelocity;
//
//			}
//		}
	}
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
