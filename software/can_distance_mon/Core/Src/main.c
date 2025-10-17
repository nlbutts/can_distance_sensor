/* USER CODE BEGIN Header */
//#pragma GCC optimize("O0")
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "FreeRTOS_CLI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "custom_ranging_sensor.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAJOR_VER 1
#define MINOR_VER 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WPILIB_DEVICE_TYPE 	10 << 24
#define WPILIB_MFG_CODE 	20 << 16
#define WPILIB_API_CLASS	5 << 10
#define WPILIB_API_INDEX    0 << 6
#define WPILIB_DEV_NUM		0
#define WPILIB_CAN_ID WPILIB_DEVICE_TYPE | WPILIB_MFG_CODE | WPILIB_API_CLASS | WPILIB_API_INDEX | WPILIB_DEV_NUM
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi2;

/* Definitions for TOF */
osThreadId_t TOFHandle;
const osThreadAttr_t TOF_attributes = {
  .name = "TOF",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1500 * 4
};
/* Definitions for CLI */
osThreadId_t CLIHandle;
const osThreadAttr_t CLI_attributes = {
  .name = "CLI",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 768 * 4
};
/* Definitions for Stats */
osThreadId_t StatsHandle;
const osThreadAttr_t Stats_attributes = {
  .name = "Stats",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* USER CODE BEGIN PV */
osThreadId_t cliTaskHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void vCommandConsoleTask(void *argument);
void vTaskPrintStats(void *argument);

/* USER CODE BEGIN PFP */
#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   100
#define WELCOME_LEN 200
static char pcWelcomeMessage[WELCOME_LEN];

uint8_t enableDebugPrint = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* This function implements the behaviour of a command, so must have the correct
   prototype. */

static void FreeRTOS_Print(char * str)
{
	uint8_t status;
	do
	{
		status = CDC_Transmit_FS((uint8_t*)str, strlen(str));
	} while (status == USBD_BUSY);
}

static BaseType_t prvPrintDistance( char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString )
{
  enableDebugPrint = !enableDebugPrint;

  return pdFALSE;
}

/* Configure DWT cycle counter for run-time stats */
void configureTimerForRunTimeStats(void)
{
  /* Enable trace and DWT if not already enabled */
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  /* Reset and enable cycle counter */
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Return current run-time counter value (cycles) */
unsigned long getRunTimeCounterValue(void)
{
	uint32_t temp = DWT->CYCCNT / (SystemCoreClock / 1000000);
  return temp;
}

static const CLI_Command_Definition_t xDebugCommand =
{
    "debug",
    "\r\ndebug: Print distance measurements\r\n",
    prvPrintDistance,
    0
};

static BaseType_t prvPrintSoftwareVer( char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString )
{

  FreeRTOS_Print(pcWelcomeMessage);

  return pdFALSE;
}

static const CLI_Command_Definition_t xVersionCommand =
{
    "ver",
    "\r\nver: Print software version\r\n",
    prvPrintSoftwareVer,
    0
};

static BaseType_t prvJumpToDFU(char *pcWriteBuffer,
                               size_t xWriteBufferLen,
                               const char *pcCommandString)
{
    __disable_irq();  // Disable all interrupts
	uint32_t * magic_address = (uint32_t*)0x10000000;
	*magic_address = 0xDEADBEEF;
	NVIC_SystemReset();
}

static const CLI_Command_Definition_t xJumpDFU =
{
  "dfu",
  "\r\ndfu: Jump to DFU programming\r\n",
  prvJumpToDFU,
  0
};

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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_FDCAN1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // Enable the CAN transceiver
  HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, 0);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
  HAL_FDCAN_Start(&hfdcan1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of TOF */
  TOFHandle = osThreadNew(StartDefaultTask, NULL, &TOF_attributes);

  /* creation of CLI */
  CLIHandle = osThreadNew(vCommandConsoleTask, NULL, &CLI_attributes);

  /* creation of Stats */
  StatsHandle = osThreadNew(vTaskPrintStats, NULL, &Stats_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  snprintf(pcWelcomeMessage, WELCOME_LEN,  "TOF DIstance Sensor V%d.%d.\r\nType Help to view a list of registered commands.\r\n", (int)MAJOR_VER, (int)MINOR_VER);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  FreeRTOS_CLIRegisterCommand(&xDebugCommand);
  FreeRTOS_CLIRegisterCommand(&xJumpDFU);
  FreeRTOS_CLIRegisterCommand(&xVersionCommand);
  MX_USB_Device_Init();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  // Configure a standard ID filter to accept all messages
  FDCAN_FilterTypeDef filterConfig;
  filterConfig.IdType = FDCAN_EXTENDED_ID;
  filterConfig.FilterIndex = 0;
  filterConfig.FilterType = FDCAN_FILTER_MASK;
  filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filterConfig.FilterID1 = 0x000;
  filterConfig.FilterID2 = 0x000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */

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
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_STB_Pin */
  GPIO_InitStruct.Pin = CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_XSHUT_Pin */
  GPIO_InitStruct.Pin = TOF_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_XSHUT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static char tempbuf[100];
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */
  uint8_t txData[8];
  FDCAN_TxHeaderTypeDef txHeader;
  txHeader.Identifier = WPILIB_CAN_ID;
  txHeader.IdType = FDCAN_EXTENDED_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = 8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  RANGING_SENSOR_Result_t result;

  FDCAN_ErrorCountersTypeDef errs;

  CUSTOM_RANGING_SENSOR_Init(0);
  RANGING_SENSOR_ProfileConfig_t profile =
  {
    .RangingProfile = VL53L3CX_PROFILE_SHORT,
    .TimingBudget = 16,
    .EnableAmbient = 0,
    .EnableSignal = 0,
  };
  CUSTOM_RANGING_SENSOR_ConfigProfile(0, &profile);
  CUSTOM_RANGING_SENSOR_Start(0, RS_MODE_BLOCKING_CONTINUOUS);

  /* Infinite loop */
  for(;;)
  {
    osDelay(50);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    if (CUSTOM_RANGING_SENSOR_GetDistance(0, &result) == BSP_ERROR_NONE)
    {
      // Format into a message
      txData[0] = result.ZoneResult[0].Status[0] & 0xFF;
      txData[1] = (result.ZoneResult[0].Distance[0] >> 8) & 0xFF;
      txData[2] = result.ZoneResult[0].Distance[0] & 0xFF;
      txData[3] = ((uint32_t)(result.ZoneResult[0].Ambient[0] * 1000) >> 8) & 0xFF;
      txData[4] = ((uint32_t)(result.ZoneResult[0].Ambient[0] * 1000)) & 0xFF;
      txData[5] = ((uint32_t)(result.ZoneResult[0].Signal[0] * 1000) >> 8) & 0xFF;
      txData[6] = ((uint32_t)(result.ZoneResult[0].Signal[0] * 1000)) & 0xFF;
      txData[7] = MAJOR_VER << 4 | MINOR_VER;
      HAL_FDCAN_Start(&hfdcan1);
      osDelay(5);
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, &txData);
//      if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
//      {
//    	  FreeRTOS_Print("rx CAN\r\n");
//      }

//      if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &errs) == HAL_OK)
//	  {
//    	  snprintf(tempbuf, 100, "TxCnt: %d RxCnt: %d Passive: %d log: %d\r\n", errs.TxErrorCnt, errs.RxErrorCnt, errs.RxErrorPassive, errs.ErrorLogging);
//    	  FreeRTOS_Print(tempbuf);
//	  }

      if (enableDebugPrint)
      {
    	  snprintf(tempbuf, 100, "Targets: %lu Status: %ld Distance: %ld\r\n",
    			  (long)result.ZoneResult[0].NumberOfTargets,
 	              (long)result.ZoneResult[0].Status[0],
 	              (long)result.ZoneResult[0].Distance[0]);
    	  FreeRTOS_Print(tempbuf);
      }
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vCommandConsoleTask */
/**
* @brief Function implementing the CLI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vCommandConsoleTask */
void vCommandConsoleTask(void *argument)
{
  /* USER CODE BEGIN vCommandConsoleTask */
  //Peripheral_Descriptor_t xConsole;
  int8_t cRxedChar, cInputIndex = 0;
  BaseType_t xMoreDataToFollow;
  /* The input and output buffers are declared static to keep them off the stack. */
  static int8_t pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];

   /* This code assumes the peripheral being used as the console has already
      been opened and configured, and is passed into the task as the task
      parameter. Cast the task parameter to the correct type. */
   //xConsole = ( Peripheral_Descriptor_t ) pvParameters;

   /* Send a welcome message to the user knows they are connected. */
  osDelay(1000);

  FreeRTOS_Print(pcWelcomeMessage);

  for( ;; )
  {
    /* This implementation reads a single character at a time. Wait in the
      Blocked state until a character is received. */
    osDelay(10);
      cRxedChar = CDC_getChar();

    if( cRxedChar == '\r' )
    {
        /* A newline character was received, so the input command string is
          complete and can be processed. Transmit a line separator, just to
          make the output easier to read. */
      char * newline = "\r\n";
      FreeRTOS_Print(newline);

        /* The command interpreter is called repeatedly until it returns
          pdFALSE. See the "Implementing a command" documentation for an
          exaplanation of why this is. */
      do
      {
            /* Send the command string to the command interpreter. Any
              output generated by the command interpreter will be placed in the
              pcOutputString buffer. */
        xMoreDataToFollow = FreeRTOS_CLIProcessCommand
                      (
                          (const char *)pcInputString,   /* The command string.*/
                          (char*)pcOutputString,  /* The output buffer. */
                          MAX_OUTPUT_LENGTH/* The size of the output buffer. */
                      );

        /* Write the output generated by the command interpreter to the
          console. */
        FreeRTOS_Print((char*)pcOutputString);
      } while( xMoreDataToFollow != pdFALSE );

        /* All the strings generated by the input command have been sent.
          Processing of the command is complete. Clear the input string ready
          to receive the next command. */
      cInputIndex = 0;
      memset( pcInputString, 0x00, MAX_INPUT_LENGTH );
    }
    else
    {
        /* The if() clause performs the processing after a newline character
          is received. This else clause performs the processing if any other
          character is received. */

      if( cRxedChar == '\n' )
      {
          /* Ignore carriage returns. */
      }
      else if( cRxedChar == '\b' )
      {
        /* Backspace was pressed. Erase the last character in the input
          buffer - if there are any. */
        if( cInputIndex > 0 )
        {
            cInputIndex--;
            pcInputString[ cInputIndex ] = ' ';
        }
      }
      else if (cRxedChar > 0)
      {
        /* A character was entered. It was not a new line, ba"\r\n", strlen( "\r\n" ); return, so it is accepted as part of the input and
          placed into the input buffer. When a n is entered the complete
          string will be passed to the command interpreter. */
        if( cInputIndex < MAX_INPUT_LENGTH )
        {
          pcInputString[ cInputIndex ] = cRxedChar;
          cInputIndex++;
          char str[2] = {cRxedChar, 0};
          FreeRTOS_Print(str);
        }
      }
    }
  }
  /* USER CODE END vCommandConsoleTask */
}

/* USER CODE BEGIN Header_vTaskPrintStats */
/**
* @brief Function implementing the Stats thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskPrintStats */
void vTaskPrintStats(void *argument)
{
  /* USER CODE BEGIN vTaskPrintStats */
  TaskStatus_t *pxTaskStatusArray;
  UBaseType_t *prevTaskIDArray;
  uint32_t *prevTaskTimeArray;
  UBaseType_t uxArraySize, x, y;
  uint32_t ulTotalRunTime; // FreeRTOS also returns total time, but we don't rely on it for MS calc
  
  // Get the number of tasks to allocate the array size
  uxArraySize = uxTaskGetNumberOfTasks();

  // Allocate an array to hold the TaskStatus_t structures
  pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
  // Allocate an array to hold the previous task run times
  prevTaskIDArray = pvPortMalloc( uxArraySize * sizeof( UBaseType_t ) );
  // Allocate an array to hold the previous task run times
  prevTaskTimeArray = pvPortMalloc( uxArraySize * sizeof( uint32_t ) );

  for (int i = 0; i < uxArraySize; i++)
  {
	  prevTaskIDArray[i] = i;
  }

  /* Infinite loop */
  for(;;)
  {
    osDelay(5000);

    if( pxTaskStatusArray != NULL )
    {
      // Get the task information. ulTotalRunTime is set to the total time the
      // system has been running since stats collection started.
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

      FreeRTOS_Print("----------------------------------------------------------------------\r\n");
      FreeRTOS_Print("Task Name\tTotal Time (us)\t\tPercent (%)\tStack (bytes)\r\n");
      FreeRTOS_Print("----------------------------------------------------------------------\r\n");

      // Iterate through each task's status
      for( x = 0; x < uxArraySize; x++ )
      {
          // ulRunTimeCounter is the accumulated time in Run-Time Clock Ticks.
          uint32_t ulTaskTicks = pxTaskStatusArray[ x ].ulRunTimeCounter;
          uint32_t diffTime;

          for (y = 0; y < uxArraySize; y++)
          {
        	  if (prevTaskIDArray[y] == pxTaskStatusArray[x].xTaskNumber)
        	  {
        		  diffTime = ulTaskTicks - prevTaskTimeArray[y];
        		  prevTaskTimeArray[y] = ulTaskTicks;
        		  break;
        	  }
          }

          int percent = diffTime / 50000;

          // Print the result. Note the use of %s for name and %llu for uint64_t
          snprintf(tempbuf, 100, "%-16s\t%d\t\t%d\t\t%d\r\n",
                  pxTaskStatusArray[ x ].pcTaskName, 
                  (int)diffTime,
				  percent,
				  (int)pxTaskStatusArray[x].usStackHighWaterMark);
          FreeRTOS_Print(tempbuf);
      }
      
      FreeRTOS_Print("----------------------------------------------------------------------\r\n");
    }
    else
    {
    	FreeRTOS_Print("ERROR: Failed to allocate memory for task status array.\r\n");
    }
  }
  /* USER CODE END vTaskPrintStats */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
