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
#include "robomaster.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Que{
	float data[16];
	uint8_t pointer;
	uint8_t size;
}Que;
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

// ???��?��??��?��?スト用
uint32_t id;
uint32_t dlc;
uint32_t data[8];
//int16_t omega;
int16_t torque;
Que mean;

// ロボマス用構造体宣言
RobomasterTypedef Robomaster[4];

// CAN受信コールバック関数
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

			Robomaster_RxCAN(&Robomaster[id - 1], &RxData[0]);

			//calc moving average
			mean.data[mean.pointer] = data[2] << 8 | data[3];
			mean.pointer = (mean.pointer==mean.size-1) ? 0u : mean.pointer+1;
			for(uint8_t i=0u; i<mean.size; i++){
				Robomaster[id - 1].AngularVelocity += mean.data[i];
			}
			Robomaster[id - 1].AngularVelocity /= (float)mean.size;

			// 送信
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) {
				// 送信用構�??体�????��?��??��?��定義
				CAN_TxHeaderTypeDef TxHeader;
				// IDの設???��?��??��?��?
				TxHeader.StdId = 0x200;
				// 標準IDを使用
				TxHeader.IDE = CAN_ID_STD;
				// ???��?��??��?��?ータフレー???��?��??��?��? or リモートフレー???��?��??��?��?
				TxHeader.RTR = CAN_RTR_DATA;
				// ???��?��??��?��?ータ長???��?��??��?��? [byte]
				TxHeader.DLC = 8;
				// タイ???��?��??��?��?スタン???��?��??��?��?
				TxHeader.TransmitGlobalTime = DISABLE;
				// 8byteの送信???��?��??��?��?ータ
				uint8_t TxData[8] = { 0 };
				for (int i = 0; i < 4; i++) {
					TxData[2 * i] = Robomaster[i].TargetTorque >> 8;
					TxData[2 * i + 1] = Robomaster[i].TargetTorque & 0x00FF;
				}
				// 送信に使ったTxMailboxが�????��?��??��?��納される
				uint32_t TxMailbox;
				// メ???��?��??��?��?セージ送信
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
	mean.size = 10;
	mean.pointer = 0u;
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

	// フィルタの構造体宣言
	CAN_FilterTypeDef filter;
	// IDとMaskの変数宣言
	uint32_t fid;
	uint32_t fmask;

	/* CAN2 FIFO0 (For Robomaster) */
	// ID and Mask Register
	fid = 0x200;
	fmask = 0x7F0;
	// CAN2のFilter Bankは14から
	filter.SlaveStartFilterBank = 14;
	// Filter Bank 14に設定開始
	filter.FilterBank = 14;
	// For FIFO0
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	// ID and Mask
	filter.FilterIdHigh = fid << 5;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh = fmask << 5;
	filter.FilterMaskIdLow = 0;
	// Filter適用
	HAL_CAN_ConfigFilter(&hcan2, &filter);

	/* CAN2 FIFO1 (For Encoder) */
	// ID and Mask Register
	fid = 0x400;
	fmask = 0x7F0;
	// CAN2のFilter Bankは14から
	filter.SlaveStartFilterBank = 14;
	// Filter Bank 15に設定開始
	filter.FilterBank = 15;
	// For FIFO1
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	// ID and Mask Bit Configure
	filter.FilterIdHigh = fid << 5;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh = fmask << 5;
	filter.FilterMaskIdLow = 0;
	// Filter適用
	HAL_CAN_ConfigFilter(&hcan2, &filter);

	// CAN2 Start
	HAL_CAN_Start(&hcan2);
	// CAN2 FIFO0 and FIFO1 Enable Interrupt
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

	// ゲイン設定
	float32_t Kp = 10;
	float32_t Ki = 0.00;
	float32_t Kd = 0.00;
	float32_t f_i = 50.0f;	//for feedforwared
	float32_t f_j = 5.0f;	//for feedforwared
	for (int i = 0; i < 4; i++) {
		// Robomaster Initialize
		memset(&Robomaster[i], 0, sizeof(RobomasterTypedef));
		// PID Initialize
		Robomaster[i].PID.Kp = Kp;
		Robomaster[i].PID.Ki = Ki;
		Robomaster[i].PID.Kd = Kd;
		arm_pid_init_f32(&Robomaster[i].PID, 1);
	}

	/* Configure UDP */
	// Data Buffer For UDP
	int16_t rxbuf[16] = { 0 };
	int16_t txbuf[16] = { 0 };
	//アドレスを宣?��?
	struct sockaddr_in rxAddr, txAddr;
	//ソケ?��?トを作�??
	int socket = lwip_socket(AF_INET, SOCK_DGRAM, 0);
	//アドレスのメモリを確?��?
	memset((char*) &txAddr, 0, sizeof(txAddr));
	memset((char*) &rxAddr, 0, sizeof(rxAddr));
	//アドレスの構�??体�??��?��?ータを定義
	rxAddr.sin_family = AF_INET; //プロトコルファミリの設?��?(IPv4に設?��?)
	rxAddr.sin_len = sizeof(rxAddr); //アドレスの?��?ータサイズ
	rxAddr.sin_addr.s_addr = INADDR_ANY; //アドレスの設?��?(今回はすべてのアドレスを受け�??��れるためINADDR_ANY)
	rxAddr.sin_port = lwip_htons(PC_PORT); //ポ�??��ト�??��?��??��?
	txAddr.sin_family = AF_INET; //プロトコルファミリの?��??��?(IPv4に設?��?)
	txAddr.sin_len = sizeof(txAddr); //アドレスの?��?ータのサイズ
	txAddr.sin_addr.s_addr = inet_addr(PC_ADDR); //アドレスの設?��?
	txAddr.sin_port = lwip_htons(PC_PORT); //ポ�??��ト�??��?��??��?
	// whileでbindを�?ってみると�?まく行く可能性�?
//	(void) lwip_bind(socket, (struct sockaddr*) &rxAddr, sizeof(rxAddr)); //IPアドレスとソケ?��?トを紐付けて受信をできる状態に
	while(lwip_bind(socket, (struct sockaddr*) &rxAddr, sizeof(rxAddr)) < 0) {

	}
	socklen_t n; //受信した?��?ータのサイズ
	socklen_t len = sizeof(rxAddr); //rxAddrのサイズ


	// ARP待ち
//	HAL_Delay(700);

	/* Infinite loop */
	for (;;) {
		lwip_sendto(socket, (uint8_t*) txbuf, sizeof(txbuf), 0,
				(struct sockaddr*) &txAddr, sizeof(txAddr)); //受信したら�??��信する
		n = lwip_recvfrom(socket, (uint8_t*) rxbuf, sizeof(rxbuf), (int) NULL,
				(struct sockaddr*) &rxAddr, &len); //受信処?��?(blocking)

		for(int i = 0; i < 4; i++) {
			Robomaster[i].TargetAngularVelocity = (float32_t)rxbuf[i] * 19 / 100;
			txbuf[i] = Robomaster[i].AngularVelocity * 100;
		}

//		shoki = buff[8];
//		uator = buff[9];
//		functions = buff[10];

//		sort(rxbuf,TagetAngularVelocity);


		// モーターの速度制御
		for (int i = 0; i < 4; i++) {
			if (Robomaster[i].Event == 1) {
				// 誤差e[n]の計算
				Robomaster[i].AngularVelocityError = Robomaster[i].TargetAngularVelocity - (float32_t)Robomaster[i].AngularVelocity;
				// PID Controller
//				Robomaster[i].PID.state[2] = 0.0f;
				Robomaster[i].TargetTorque = (int16_t)arm_pid_f32(&Robomaster[i].PID, Robomaster[i].AngularVelocityError) + (f_i+f_j)*Robomaster[i].TargetAngularVelocity - f_i*Robomaster[i].PreTargetAngularVelocity;
//				Robomaster[i].TargetTorque = (int16_t)(Robomaster[i].AngularVelocityError * Robomaster[i].PID.Kp);
				Robomaster[i].PreTargetAngularVelocity = Robomaster[i].TargetAngularVelocity;
			}
		}
//		int16_t test = Robomaster[0].Angle;z
		//モーターの速度制御
//		for (int i = 0; i < 4; i++) {
//			if (Robomaster[i].Event == 1) {
//				Robomaster[i].Event = 0;
//
//				TagetAngularVelocity[i] = rxbuf[i];
//
//				difference = TagetAngularVelocity[i] - Robomaster[i].AngularVelocity;
//
//				//pi制御
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
