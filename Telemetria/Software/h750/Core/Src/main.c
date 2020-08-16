/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "SPI_SX.h"

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

CRC_HandleTypeDef hcrc;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

QueueHandle_t xQBufferPointer, xQTransmissonRequest;


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
  MX_CRC_Init();
  MX_QUADSPI_Init();
  MX_USART1_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 1024);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 512);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 3;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_QSPI;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 320;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 16;
  PeriphClkInitStruct.PLL2.PLL2R = 4;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 4;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

  hcrc.Init.CRCLength = CRC_POLYLENGTH_32B;

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  hsd1.Init.TranceiverPresent = SDMMC_TRANSCEIVER_NOT_PRESENT;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  TIM5->SR = 0;

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, NRST_Pin|RXEN_Pin|TXEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : NRST_Pin RXEN_Pin TXEN_Pin */
  GPIO_InitStruct.Pin = NRST_Pin|RXEN_Pin|TXEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_SS_Pin */
  GPIO_InitStruct.Pin = SPI3_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI3_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO2_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

_Bool flag = 0;

void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
{
	flag = 1;
}


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
  /* USER CODE BEGIN 5 */
//	TIM5->DIER = 0x01;
//	TIM5->PSC = 0x04;
//	TIM5->ARR = 0x1E;
//	GPIOA->BSRR = 0x040000;

  /* Infinite loop */
  for(;;)
  {
//	  TIM5->CNT = 0x00;
//	  GPIOA->BSRR = 0x00000002;
//	  osDelay(10);
//	  GPIOA->BSRR = 0x00020000;
//	  TIM5->CR1 = 0x01;
//	  osDelay(10);
//	  GPIOA->BSRR = 0x040000;

	  GPIOA->BSRR = 0x00000002;
	  osDelay(400);
	  GPIOA->BSRR = 0x00020000;
	  osDelay(400);

  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */

	uint8_t *raw_pkt = NULL;
	uint16_t id, len;
	uint32_t crc_val, crc_pkt;
	uint8_t msg[] = "SPN";
	uint16_t erros = 0;

	uint8_t TxRequestCode = 0x0F;


	xQBufferPointer = xQueueCreate(1, sizeof(uint8_t*));
	xQTransmissonRequest = xQueueCreate(1, sizeof(uint8_t));

	raw_pkt = pvPortMalloc(sizeof(uint8_t) * 2056);

	xQueueSend(xQBufferPointer, &raw_pkt, (TickType_t) 10);

	// o endereço do vetor raw_pkt deve ser enviado pra task de envio


	HAL_SPI_Receive_DMA(&hspi3, raw_pkt, 2056);

	osDelay(3000);

	HAL_UART_Transmit(&huart1, &msg[0], 1, HAL_MAX_DELAY);

  /* Infinite loop */
  for(;;)
  {

	  while(flag != 1)	{ osDelay(1); }
	  flag = 0;

	  id = (raw_pkt[0] << 8) | raw_pkt[1];
	  len = (raw_pkt[2] << 8) | raw_pkt[3];
	  crc_pkt = (raw_pkt[len+4] << 24) | (raw_pkt[len+5] << 16) | (raw_pkt[len+6] << 8) | (raw_pkt[len+7]);

	  crc_val = HAL_CRC_Calculate(&hcrc, &raw_pkt[0], len + 4);
	  crc_val = crc_val ^ 0xFFFFFFFF;

	  if(crc_pkt == crc_val)
	  {

		 // informa que uma mensagem do tipo foto está pronta para ser enviada
		 // suspende a tarefa, ela volta a ser liberada quando o envio terminar

		  xQueueSend(xQTransmissonRequest, (void*)&TxRequestCode, (TickType_t) 10);

		  vTaskResume(myTask03Handle);
		  vTaskSuspend(myTask02Handle);


		  HAL_UART_Transmit(&huart1, &msg[2], 1, HAL_MAX_DELAY);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart1, &msg[1], 1, HAL_MAX_DELAY);
		  erros++;
	  }

	osDelay(1);

	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */

	TIM5->DIER = 0x01;		// Update interrupt enable
	TIM5->PSC = 0x04;		// Clock prescaler			APB1_Timer_clock/2
	TIM5->ARR = 0x3C;		// Auto-reload register

	uint16_t id, id0, len;
	uint8_t TxRequest;

	HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NRST_GPIO_Port, NRST_Pin, GPIO_PIN_SET);

	uint8_t rsp[10];

	uint8_t msg[] = "teste teste 666";
	uint8_t *ptr_msg = NULL, *ptr_buffer = NULL;


	uint8_t SetStandby[2] = {0x80, 0x00};		// RC
	uint8_t SetDIO3AsTCXOCtrl[5] = {0x97, 0x07, 0x00, 0x00, 0x32};
	uint8_t CalibrateImage[3] = {0x98, 0xE1, 0xE9};
	uint8_t ClearDeviceErrors[3] = {0x07, 0, 0};
	uint8_t GetDeviceErrors[4] = {0x17, 0, 0, 0};
	uint8_t SetPacketType[2] = {0x8A, 0x01};
	uint8_t SetFrequency[5] = {0x86, 0x39, 0x30, 0x00, 0x00};
	uint8_t SetPaConfig[5] = {0x95, 0x02, 0x02, 0x00, 0x01};
	uint8_t SetTxParam[3] = {0x8E, 0x0E, 0x03};		// 14dBm
	uint8_t SetBufferBaseAddress[3] = {0x8F, 0x00, 0x00};

			// WriteBuffer

			xQueueReceive(xQBufferPointer, &ptr_buffer, (TickType_t) 1000);
			vQueueDelete(xQBufferPointer);



			// a queue descarrega aqui, ptr_msg vai ter até 256 bytes + 2, o registrador payload lenght tbm precisa ser ajustado
			/*
			 * 			ptr_msg = pvPortMalloc(258)
			 *
			 * 			total_pkt = len/256     normalmente vao ser 8
			 *
			 * 			tamanho do pacotinho = 256
			 *
			 * 			for(int i = 0; i < total_pkt; i++)
			 *
			 * 				if(i == total_pkt)
			 * 					tamanho do pacotinho = len - 256*i
			 *
			 * 				for(int j = 0; j < tamanho do pacotinho; j++)
			 * 					*(ptr_msg + offset) = raw_pkt[j + 256*i]
			 *
			 * 				preenche o registrador de tamanho do payload com o tamanho do pacotinho
			 * 				carrega a mensagem no registrador
			 * 				coloca no modo TX
			 * 				aguarda a interrupção e solicita novo envio
			 */

			ptr_msg = pvPortMalloc(sizeof(uint8_t) * 258);
			*ptr_msg = 0x0E;
			*(ptr_msg + 1) = 0x00;						// offset
//			for(int i = 0; i < sizeof(msg); i++)
//			{
//				*(ptr_msg + i + 2) = msg[i];
//			}

	uint8_t SetModulationParams[9] = {0x8B, 0x05, 0x06, 0x01, 0x00, 0, 0, 0, 0};
	uint8_t SetPacketParam[10] = {0x8C, 0x00, 0x08, 0x00, 0xFF, 0x01, 0x00, 0, 0, 0};
	uint8_t Workaround1[5] = {0x1D, 0x08, 0x89, 0, 0};
	uint8_t Workaround2[5] = {0x1D, 0x08, 0xD8, 0, 0};
	uint8_t SetDioIrqParams[9] = {0x08, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
	uint8_t SyncWord[5] = {0x0D, 0x07, 0x40, 0x47, 0x76};
	uint8_t SetTx[4] = {0x83, 0x00, 0x00, 0x00};

	uint8_t GetIrqStatus[4] = {0x12, 0, 0, 0};
	uint8_t ClearIrqStatus[3] = {0x2, 0x00, 0x01};

	osDelay(100);

	opcodeTx(SetStandby, 2);
	//----------------------------------
	opcodeTx(ClearDeviceErrors, 3);
	opcodeTx(SetDIO3AsTCXOCtrl, 5);
	opcodeTx(CalibrateImage, 3);
	opcodeTx(GetDeviceErrors, 4);
	// criar mecanismo para tratar possivel erro
	//----------------------------------
	opcodeTx(SetPacketType, 2);
	opcodeTx(SetFrequency, 5);
	opcodeTx(SetPaConfig, 5);
	opcodeTx(SetTxParam, 3);
	opcodeTx(SetBufferBaseAddress, 3);

	opcodeTx(SetModulationParams, 9);
	if(SetPacketType[1] == 0x01) {				// LoRa
		if(SetModulationParams[2] == 0x06)		// BW = 500 KHz
		{
			opcodeTxRx(Workaround1, rsp, 5);		// read register
			rsp[1] = 0x0D; rsp[2] = 0x08; rsp[3] = 0x89; rsp[4] = rsp[4] & 0xFB;
			opcodeTx(&rsp[1], 4);				// write register
		}
	}
	else {
		opcodeTxRx(Workaround1, rsp, 5);			// read register
		rsp[1] = 0x0D; rsp[2] = 0x08; rsp[3] = 0x89; rsp[4] = rsp[4] | 0x04;
		opcodeTx(&rsp[1], 4);					// write register
	}
	opcodeTx(SetPacketParam, 10);
	opcodeTxRx(Workaround2, rsp, 5);
	rsp[1] = 0x0D; rsp[2] = 0x08; rsp[3] = 0xD8; rsp[4] = rsp[4] | 0x1E;
	opcodeTx(&rsp[1], 4);
	opcodeTx(SetDioIrqParams, 9);
	opcodeTx(SyncWord, 5);

	do {
		xQueueReceive(xQTransmissonRequest, &TxRequest, (TickType_t) 1000);
	}while (TxRequest != 0x0F);


	id0 = ( *ptr_buffer << 8) | *(ptr_buffer+1);		// quantos pacotes vão vir, ID do primeiro pacote

	for(int i = 0; i < id0; i++) {

		len = ( *(ptr_buffer+2) << 8) | *(ptr_buffer+3);	// quantos bytes tem nesse pacote, normalmente é 2048

		int len_quo = len / 255;
		int len_rem = len % 255;

		for(int j = 0; j < len_quo; j++) {

			for(int k = 0; k < 255; k++) {
				*(ptr_msg + 2 + k) = *(ptr_buffer + 4 + k + (j*255) );
			}

			opcodeTx(ptr_msg, 258);
			opcodeTx(SetTx, 4);
			vTaskSuspend(myTask03Handle);

			opcodeTxRx(GetIrqStatus, rsp, 4);

			if((rsp[3] & 0x01) == 1)
			{
				// sucess
			} else {
				// fail
			}

			opcodeTx(ClearIrqStatus, 3);

		}


		if(len_rem > 0)
		{
			for(int k = 0; k < len_rem; k++) {
				*(ptr_msg + 2 + k) = *(ptr_buffer + 4 + k + (len_quo*255) );
			}

			SetPacketParam[4] = len_rem;
			opcodeTx(SetPacketParam, 10);
			opcodeTx(ptr_msg, 258);
			opcodeTx(SetTx, 4);
			vTaskSuspend(myTask03Handle);

			opcodeTxRx(GetIrqStatus, rsp, 4);

			if((rsp[3] & 0x01) == 1)
			{
				// sucess
			} else {
				// fail
			}

			opcodeTx(ClearIrqStatus, 3);

		}

		vTaskResume(myTask02Handle);
		vTaskSuspend(myTask03Handle);


	}


//	opcodeTx(SetPacketParam, 10);
//	opcodeTx(ptr_msg, sizeof(msg)+2);
//	opcodeTx(SetTx, 4);
//	vTaskSuspend(myTask03Handle);
//
//	opcodeTxRx(GetIrqStatus, rsp, 4);
//
//	if((rsp[3] & 0x01) == 1)
//	{
//		// sucess
//	} else {
//		// fail
//	}
//
//	opcodeTx(ClearIrqStatus, 3);

	HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET);

  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartTask03 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
