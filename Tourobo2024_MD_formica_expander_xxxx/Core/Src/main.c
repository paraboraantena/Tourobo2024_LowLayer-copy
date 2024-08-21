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
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t id;
uint32_t dlc;
uint8_t data[8];
uint8_t failure = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		id = (RxHeader.IDE == CAN_ID_STD) ? RxHeader.StdId : RxHeader.ExtId;   // ID
		dlc = RxHeader.DLC;                                                   // DLC
		data[0] = RxData[0];                                                 // Data
		data[1] = RxData[1];
		data[2] = RxData[2];
		data[3] = RxData[3];
		data[4] = RxData[4];
		data[5] = RxData[5];
		data[6] = RxData[6];
		data[7] = RxData[7];

	} else {
		failure = failure + 1;
	}

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
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
  MX_I2C3_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh = 0;  // „Éï„Ç£„É´„Çø„ÉºID(???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ? ?16„Éì„ÉÉ ?)
	filter.FilterIdLow = 0;   // „Éï„Ç£„É´„Çø„ÉºID(???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ? ?16„Éì„ÉÉ ?)
	filter.FilterMaskIdHigh = 0; // „Éï„Ç£„É´„Çø„Éº„Éû„Çπ„ÇØ(???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ? ?16„Éì„ÉÉ ?)
	filter.FilterMaskIdLow = 0; // „Éï„Ç£„É´„Çø„Éº„Éû„Çπ„ÇØ(???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ? ?16„Éì„ÉÉ ?)
	filter.FilterScale = CAN_FILTERSCALE_32BIT;    // „Éï„Ç£„É´„Çø„Éº„Çπ„Ç±„Éº„É´
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // „Éï„Ç£„É´„Çø„Éº„Å´Ââ≤„ÇäÂΩì„Å¶„ÇãFIFO
	filter.FilterBank = 0;                        // „Éï„Ç£„É´„Çø„Éº„Éê„É≥„ÇØNo
	filter.FilterMode = CAN_FILTERMODE_IDMASK;    // „Éï„Ç£„É´„Çø„Éº„É¢„Éº ?
	filter.SlaveStartFilterBank = 0;                      // „Çπ„É¨„Éº„ÉñCAN„ÅÆÈñãÂßã„Éï„Ç£„É´„Çø„Éº„Éê„É≥„ÇØNo
	filter.FilterActivation = ENABLE;                   // „Éï„Ç£„É´„Çø„ÉºÁÑ°Âäπ?  ÊúâÂäπ
	HAL_CAN_ConfigFilter(&hcan2, &filter);

	// CAN„Çπ„Çø„Éº ?
	HAL_CAN_Start(&hcan2);
	// Ââ≤„ÇäËæº„ÅøÊúâÂäπ
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	//CANÈÄÅ‰ø°ÈñãÔøΩ?

	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t TxData[8];

	for (;;) {

		if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) {
			TxHeader.StdId = 0x201;                 // CAN ID
			TxHeader.RTR = CAN_RTR_DATA; // „Éï„É¨„Éº???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?„Çø„Ç§„ÉóÔøΩ????????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?„Éº„Çø„Éï„É¨„Éº???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?
			TxHeader.IDE = CAN_ID_STD; // Ê®ôÊ∫ñID(11????????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?)
			TxHeader.DLC = 8;             // ???????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ??????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?„Éº„ÇøÈï∑„ÅØ8„Éê„Ç§„Éà„Å´
			TxHeader.TransmitGlobalTime = DISABLE;  // ???
			TxData[0] = 0x41;
			TxData[1] = 0x42;
			TxData[2] = 0x43;
			TxData[3] = 0x44;
			TxData[4] = 0x45;
			TxData[5] = 0x46;
			TxData[6] = 0x47;
			TxData[7] = 0x48;
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
		}
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
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
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 89;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, CW2_Pin|CCW2_Pin|SOLV5_Pin|SOLV3_Pin
                          |SOLV2_Pin|SOLV1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CW3_Pin|CCW3_Pin|SOLV4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SOLV6_Pin|CCW1_Pin|CW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CW2_Pin CCW2_Pin SOLV5_Pin SOLV3_Pin
                           SOLV2_Pin SOLV1_Pin */
  GPIO_InitStruct.Pin = CW2_Pin|CCW2_Pin|SOLV5_Pin|SOLV3_Pin
                          |SOLV2_Pin|SOLV1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC2A_Pin ENC2B_Pin ENC3A_Pin ENC3B_Pin */
  GPIO_InitStruct.Pin = ENC2A_Pin|ENC2B_Pin|ENC3A_Pin|ENC3B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CW3_Pin CCW3_Pin SOLV4_Pin */
  GPIO_InitStruct.Pin = CW3_Pin|CCW3_Pin|SOLV4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SOLV6_Pin CCW1_Pin CW1_Pin */
  GPIO_InitStruct.Pin = SOLV6_Pin|CCW1_Pin|CW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1A_Pin ENC1B_Pin */
  GPIO_InitStruct.Pin = ENC1A_Pin|ENC1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
