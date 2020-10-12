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
#include "SPI_SX.h"
#include "stdio.h"
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

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

osThreadId triagemTaskHandle;
/* USER CODE BEGIN PV */

// ###  FreeRTOS  ###

TaskHandle_t xHandleTransmitter = NULL;
TaskHandle_t xHandleDeploy = NULL;
TaskHandle_t xHandleImage = NULL;

SemaphoreHandle_t xMutexSpiRadio;
SemaphoreHandle_t xSemBinRadio, xSemBinRX;
SemaphoreHandle_t xSemBinRadioImage;
SemaphoreHandle_t xSemBinImgUART;
SemaphoreHandle_t xSemBinImgSPI;

QueueHandle_t xQueueRadio = NULL;
QueueHandle_t xQueueBlockFeedback = NULL;
QueueHandle_t xQueueRepeatMsg = NULL;

TimerHandle_t xTimerTriagemWatchdog;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
void TriagemTask(void const * argument);

/* USER CODE BEGIN PFP */

// ###  FreeRTOS  ###

void vTaskTransmitter( void * pvParameters );
void vTaskDeploy( void * pvParameters );
void vTaskImage( void * pvParameters );
void vTimerCallback( TimerHandle_t xTimer );

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

  vTraceEnable(TRC_START);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  BaseType_t xReturned;

  /* Task Transmitter, equivalente a task 1 do modelo,
   *  esta task configura o radio e transmite os dados
   *  periodicamente */

  /* Create the task, storing the handle. */
  xReturned = xTaskCreate(
                  vTaskTransmitter,       	/* Function that implements the task. */
                  "Transmitter",    		/* Text name for the task. */
                  300,      				/* Stack size in words, not bytes. */
                  ( void * ) 1,    			/* Parameter passed into the task. */
				  ( ( UBaseType_t ) 3U ),	/* Priority at which the task is created. */
                  &xHandleTransmitter );    /* Used to pass out the created task's handle. */

  if( xReturned != pdPASS )
  {
      /* A Task não foi criada, retornar um erro */
	  //  # IMPLEMENTAR TRATAMENTO DE ERRO #
	  configASSERT(0);
  }



  /* Task Deploy, equivalente a task 4 do modelo,
   *  esta task realiza o deploy da antena e em
   *  seguida retorna um feedback para task 1   */

  xReturned = xTaskCreate(
                  vTaskDeploy,       		/* Function that implements the task. */
                  "Deploy",    				/* Text name for the task. */
                  64,      					/* Stack size in words, not bytes. */
                  ( void * ) 1,    			/* Parameter passed into the task. */
				  ( ( UBaseType_t ) 3U ),	/* Priority at which the task is created. */
                  &xHandleDeploy );    		/* Used to pass out the created task's handle. */

    if( xReturned != pdPASS )
    {
    	/* A Task não foi criada, retornar um erro */
    	//  # IMPLEMENTAR TRATAMENTO DE ERRO #
    	configASSERT(0);
    }



    /* Task Image, equivalente a task 2 do modelo,
     *  esta task faz a aquisição de um pedaço da
     *  imagem capturada pelo Raspberry */

    /* Create the task, storing the handle. */
    xReturned = xTaskCreate(
                    vTaskImage,       		/* Function that implements the task. */
                    "Image",    			/* Text name for the task. */
                    1700,      				/* Stack size in words, not bytes. */
                    ( void * ) 1,    		/* Parameter passed into the task. */
  				  ( ( UBaseType_t ) 2U ),	/* Priority at which the task is created. */
                    &xHandleImage );   /* Used to pass out the created task's handle. */

    if( xReturned != pdPASS )
    {
        /* A Task não foi criada, retornar um erro */
  	  //  # IMPLEMENTAR TRATAMENTO DE ERRO #
  	  configASSERT(0);
    }


    xTimerTriagemWatchdog= xTimerCreate
						   ( /* Just a text name, not used by the RTOS
							 kernel. */
							 "TimerTri",
							 /* The timer period in ticks, must be
							 greater than 0. */
							 50000,
							 /* The timers will auto-reload themselves
							 when they expire. */
							 pdFALSE,
							 /* The ID is used to store a count of the
							 number of times the timer has expired, which
							 is initialised to 0. */
							 ( void * ) 0,
							 /* Each timer calls the same callback when
							 it expires. */
							 vTimerCallback
						   );

    if( xTimerTriagemWatchdog == NULL )
	{
	 /* The timer was not created. */
    	configASSERT(0);
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
  /* definition and creation of triagemTask */
  osThreadDef(triagemTask, TriagemTask, osPriorityNormal, 0, 400);
  triagemTaskHandle = osThreadCreate(osThread(triagemTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 320;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 16;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim5.Init.Period = 4294967295;
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
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : SPI3_NSS_Pin */
  GPIO_InitStruct.Pin = SPI3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI3_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Fdb_Deploy_Pin */
  GPIO_InitStruct.Pin = Fdb_Deploy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Fdb_Deploy_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO2_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


/* Esta struct é a forma padronizada que as tasks vão informar um feedback */
typedef struct
{
	/* ID ou TAG para associar a task*/
	uint8_t ID_Task[3];

	/* Parametro a ser passado como feedback */
	uint8_t parameter[2];

} feedback;


/* Enum para tornar o codigo mais legivel */
enum status_deploy { folded = 0x30, deploying, not_deployed, fully_deployed } status_dpl;

/* Variavel do feedback do status do deploy, precisou ser global pq usa na interrupção */
feedback deploy, image;



void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	/* Usado conforme recomendação da API */
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;


	switch (GPIO_Pin) {
		case Fdb_Deploy_Pin:

			/*Desativa elemento de aquecimento*/
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

			/* Altera status do feedback do deploy*/
			deploy.parameter[0] = fully_deployed;
			break;

		case DIO1_Pin:

			/* Interrupção TX DONE */

			/* Limpa flags*/
			SX_ClearIrqStatus(0x01);

			/* Desativa o amplificador de RF (TX) para poupar (MUITA) energia */
			HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET);

			/* Este semaforo indica que a transmissão de dados finalizou, e agora o
			 *   modo de recepção pode começar */
			xSemaphoreGiveFromISR(xSemBinRadio, &xHigherPriorityTaskWoken);

			break;

		case DIO2_Pin:

			/* Interrupção RX DONE */

			/* Limpa flags*/
			SX_ClearIrqStatus(0x02);

			/* Desativa o amplificador de RF (RX) para poupar energia */
			HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_RESET);

			/* Este semaforo libera o procedimento para ler uma nova mensagem do FIFO,
			 *   no caso, uma mensagem que acabou de chegar */
			xSemaphoreGiveFromISR(xSemBinRX, &xHigherPriorityTaskWoken);
			break;

		default:
			break;
	}

	/* Esta função foi usada conforme recomendação da API */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}




void vTaskTransmitter( void * pvParameters )
{

	/*	Esta task é responsável por reunir e enviar os dados periódicamente	 */

	int contador = 0;
	char payload[30];

	// zera a memoria
	for(int i = 0; i < sizeof(payload); i++)	payload[i] = 0x00;	//							memset ##########

	//-----------------------------------------------------------------------------------------------------------------

	/* Cria semaforo tipo Mutex para gerenciar a comunicação SPI com o rádio. */
	xMutexSpiRadio = xSemaphoreCreateMutex();

	/* Cria Queue que ira receber o ponteiro da variavel feedback das outras tarefas,
	 *   pode armazenar até 5 ponteiros simultaneamente */
	xQueueRadio = xQueueCreate( 5, sizeof(feedback *) );

	/* Verifica se o semaforo e queue foram criados */
	if( (xMutexSpiRadio == NULL) || (xQueueRadio == NULL) )
	{
		/* One or more queues were not created successfully as there was not enough
		  heap memory available.  Handle the error here.  Queues can also be created
		  statically. */
		configASSERT(0);
	}
	//-----------------------------------------------------------------------------------------------------------------

	/* -=-=-=-=-=-=-=-=- TIMER_5 SETUP -=-=-=-=-=-=-=-=- *
	 *													 *
	 * Este timer serve para dar tempo ao BUSY,			 *
	 *  especificado em datasheet, pag 52, Tsw.   		 *
	 *  												 *
	 *  Está configurado para operar com 480MHz no clock *
	 *  	principal, 240MHz em APB1_Timer_clock		 *
	 *  												 *
	 *  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= */

	TIM5->CR1 = 0x04;		// Control register, Only counter overflow/underflow generates an update interrupt or DMA request
	TIM5->SR = 0;			// Clear Flag
	TIM5->DIER = 0x01;		// Update interrupt enable
	TIM5->PSC = 0x02;		// Clock prescaler			APB1_Timer_clock/2
	TIM5->ARR = 0x1E;		// Auto-reload register

	//-----------------------------------------------------------------------------------------------------------------

	/* Tenta obter o semaforo SPI durante 1000 ticks, se não conseguir
	 *   se não conseguir, o rádio não poderá ser acessado e uma
	 *   ação deve ser tomada automaticamente */
	if( xSemaphoreTake( xMutexSpiRadio, ( TickType_t ) 1000 ) == pdTRUE )
	{
		/* We were able to obtain the semaphore and can now access the
		shared resource. */

		/* -=-=-=-=-=-=-=-=- RADIO SETUP -=-=-=-=-=-=-=-=- */

		/* Sync Word Lora*/
		uint8_t LoRaSyncWord[2] = { 0x14, 0x24 };

		/* Habilita o rádio */
		HAL_GPIO_WritePin(NRST_GPIO_Port, NRST_Pin, GPIO_PIN_SET);

		/* Aguarda um tempinho para estabilização dele*/
		osDelay(50);

		/* Modo Standby para poder realizar as configurações iniciais */
		SX_SetStandby(0x00);

		/* Rotinas de calibração */
		SX_ClearDeviceErrors();
		SX_SetDIO3AsTCXOCtrl(0x07, 0x32);
		SX_CalibrateFunction(0x7F);
		SX_CalibrateImage();

		/* Deve tratar erros de calibração caso existam*/
		switch (SX_GetDeviceErrors())
		{
			case 0x01:

				break;
			default:
				break;
		}

		/* Setup do rádio */		//  #### Para ficar mais intuitivo, utilizar #defines    ####
		SX_SetPacketType(0x01);
		SX_SetFrequency(915e6);
		SX_SetPaConfig(0x04, 0x07, 0x00);	// Afeta a potencia de saida	0x04 0x07 0x00 MAX
		SX_SetTxParam(4, 0x03);			// Afeta a potencia de saida	22 MAX
		SX_SetBufferBaseAddress(0x7F, 0x00);
		SX_SetModulationParamsLoRa(0x07, 0x04, 0x01, 0x00);
		SX_WriteRegister(0x0740, LoRaSyncWord, 2);

		/* We have finished accessing the shared resource.  Release the
		semaphore. */
		xSemaphoreGive( xMutexSpiRadio );
	}
	else
	{
		/* We could not obtain the semaphore and can therefore not access
		the shared resource safely. */

		// Por algum motivo não foi possivel adquirir o Mutex SPI
		configASSERT(0);
		// criar uma maneira de tratar a falha ou identificar o problema
	}

	/* Libera Task de triagem, já que sabemos agora que o rádio está operacional*/
	vTaskResume(triagemTaskHandle);

	//-----------------------------------------------------------------------------------------------------------------

	/* Ponteiro para receber os Feedbacks das Tasks */
	feedback *pRxFeedback;

	/* Array de ponteiro que armazena todos os endereços dos feedbacks */
	feedback *pArrayFeedback[10];

	/* Contador de feedbacks, indice para pArrayFeedback */
	static uint8_t index = 0;

	/* -=-=-=-=-=-=-=-=- LOOP RADIO -=-=-=-=-=-=-=-=- */

	for(;;)
	{
		// Solicita Mutex
		// Recebe a queue
		// Escreve os dados no buffer
		// Envia e suspende

		for(contador = 0; contador < 1000; contador++)			// 24/08/2020 - Estou criando este loop apenas para testar o transmissor, preciso ficar enviando um dado pra validar
		{

			/* Tenta por até 50 Ticks solicitar o Mutex SPI */
			if( xSemaphoreTake( xMutexSpiRadio, ( TickType_t ) 50 ) == pdTRUE )
			{
				/* Verifica se alguma Task enviou ponteiro de feedback */
				while ( xQueueReceive(xQueueRadio, &(pRxFeedback), ( TickType_t ) 0) == pdPASS )								//  Uma variavel do tipo "feedback" é um struct que possui um ID e um PARAMETRO que é
				{																												//    passado como feedback de uma task para ser transmitido para a ground station.
					/* Os ponteiros de feedback são carregados aqui */															//
					pArrayFeedback[index++] = pRxFeedback;																		//  Exemplo: A task deploy tem ID=DPL e PARAMETRO=(estado do deploy).
				}																												//
																																//  'xQueueRadio' transporta o endereco (ponteiro) da variavel de feedback que cada
				/* Mensagem que será enviada */																					//    task está utilizando.
//				sprintf(payload, "TESTE %d - Deploy: %c\n", contador, (char) (0x30 + (pArrayFeedback[0]->parameter[0]) ) );		//
				sprintf(payload, "TEST %.3d|", contador);																		//  'pArrayFeedback' armazena todos estes ponteiros. 'index' é o indice de pArrayFeedback
																																//    e tambem indica quantas variaveis de feedback existem.
				for(int i = 0, j = 0; (i < sizeof(payload)-9) && (i < index*4); i = i + 4, j++)									//
				{
					payload[9+i] = pArrayFeedback[j]->ID_Task[0];
					payload[10+i] = pArrayFeedback[j]->parameter[0];
					payload[11+i] = pArrayFeedback[j]->parameter[1];
					payload[12+i] = '|';

				}

				payload[17] = '\n';

				SX_SetStandby(0x00);

				/* Define os parametros do packet */
//				SX_SetPacketParamsLoRa(0x08, 0x00, sizeof(payload), 0x01, 0x00);
				SX_SetPacketParamsLoRa(0x08, 0x00, 18, 0x01, 0x00);

				/* Configura interrupção TX Done */
				SX_SetDioIrqParams(0x01, 0x01, 0, 0);

				SX_SetBufferBaseAddress(0x7F, 0x00);				// ################################################## adicionei isso aq, se funcionar, mantenha

				/* Escreve uma mensagem de teste no buffer */
				SX_WriteBuffer(0x7F, (uint8_t *) payload, sizeof(payload));

				/* Transmite a mensagem que estava no buffer */
				SX_TX(0);

				/* Libera Mutex do SPI1 */
				xSemaphoreGive( xMutexSpiRadio );

				/* Suspende esta task */
				vTaskSuspend(xHandleTransmitter);
			}
			else
			{
				// Por algum motivo não foi possivel adquirir o Mutex SPI
				configASSERT(0);
				// criar uma maneira de tratar a falha ou identificar o problema
			}
		}
	}

}


void vTaskDeploy( void * pvParameters )
{
	// Inicia suspensa

	/*			SEQUENCIA DE FUNCIONAMENTO
	 *
	 * 		1. Aciona Heater
	 * 		2. Aguarda (timer ou switch) (libera task)
	 * 		3. Desliga Heater
	 * 		4. Coloca o resultado numa Queue
	 * 		5. Se o resultado foi positivo, apague a task, se não, suspenda e rearme
	 *
	 */

	/* Ponteiro para enviar o endereço da variavel 'deploy', do tipo feedback */
	feedback *pDeploy = &deploy;

	deploy.ID_Task[0] = 'D'; 	deploy.ID_Task[1] = 'P'; 	deploy.ID_Task[2] = 'L';
	deploy.parameter[0] = folded;	deploy.parameter[1] = 0;

	if( xQueueSend(xQueueRadio, (void *) &pDeploy, ( TickType_t ) 100) != pdPASS)
	{
		/* Não conseguiu colocar ponteiro na queue*/
		configASSERT(0);
	}

	/* Fica suspensa até o momento do deploy ocorrer */
	vTaskSuspend(xHandleDeploy);

	for(;;)
	{
		if(deploy.parameter[0] != fully_deployed)
		{
			deploy.parameter[0] = deploying;

			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);	// Start Heater
			osDelay(10000);												// wait
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);	// Stop Heater

			if(deploy.parameter[0] == deploying)
				deploy.parameter[0] = not_deployed;
		}

		vTaskSuspend(xHandleDeploy);

	}
}

void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Informa que a mensagem do SPI está disponivel
	xSemaphoreGiveFromISR(xSemBinImgSPI, &xHigherPriorityTaskWoken);

	// Força uma mudança de contexto caso seja necessario
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{
	// libera um semaforo binario
	// isso precisa informar que recebeu um feedback

	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(xSemBinImgUART, &xHigherPriorityTaskWoken);

	// Força uma mudança de contexto caso seja necessario
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void vTimerCallback( TimerHandle_t xTimer )
{
	// ATUALIZAÇÃO, EU AINDA TENHO QUE ARRUMAR ISSO

	// Se der merda, o timer não será parado, então isso garante que volte pra um estado conhecido

//	image.parameter[0] = 0x30;
//	image.parameter[1] = 'F';
//	xSemaphoreGive(xSemBinRadio);
//	vTaskResume(triagemTaskHandle);
}

void vTaskImage( void * pvParameters )
{

	/*	FUNCIONAMENTO DESTA TASK

		1- O uC envia um sinal por UART para o RPi quando for solicitado pela task triagem
		*- Antes de um arquivo ser efetivamente transmitidos, a ground station deve estar
			ciente sobre os metadados do arquivo que ela vai receber
		*- A task do transmissor deve continuar enviando os dados para a ground station,
			de modo que esta task transmita o arquivo no restante do tempo
		*- Esta task passa como feedback para a task do transmissor a pergunta se a GS
			conseguiu compreender todos os dados
	*/

	TaskStatus_t xTaskDetails;

	feedback *pImage = &image;

	uint8_t msgRPiMetadata = 0x01,
			msgRPiBegin = 0x02,
			msgRPiNext = 0x03,
			msgRPiRepeat = 0x00,
			msgRPiReset = 0x55;

	uint8_t *pBufferSPI = NULL;
	uint8_t metadata[5] = {0,0,0,0,0};
	uint16_t ID = 0, size = 0;
	uint32_t CRC32 = 0;

	static uint8_t RepeatMsg[17];

	uint8_t BufferUART;

	uint8_t BlockFeedback = 1;

	image.ID_Task[0] = 'I'; 	image.ID_Task[1] = 'M'; 	image.ID_Task[2] = 'G';
	image.parameter[0] = 0x30; 	image.parameter[1] = 0x00;

	if( xQueueSend(xQueueRadio, (void *) &pImage, ( TickType_t ) 100) != pdPASS)
	{
		/* Não conseguiu colocar ponteiro na queue*/
		configASSERT(0);
	}

	xSemBinRadioImage 	= xSemaphoreCreateBinary();
	xSemBinImgUART 		= xSemaphoreCreateBinary();
	xSemBinImgSPI 		= xSemaphoreCreateBinary();

	xQueueBlockFeedback = xQueueCreate(1, sizeof(uint8_t));

	/* Verifica se foi possivel criar o semaforo */
	if( xSemBinRadioImage == NULL || xSemBinImgUART == NULL || xSemBinImgSPI == NULL || xQueueBlockFeedback == NULL)
	{
		//  # IMPLEMENTAR TRATAMENTO DE ERRO #
		// FALTOU MEMORIA
		/* There was insufficient FreeRTOS heap available for the semaphore to
		be created successfully. */
		configASSERT(0);
	}

	// Aloca memoria RAM para o buffer
	pBufferSPI = pvPortMalloc(4056);

	if(pBufferSPI == NULL)
	{
		// Não foi possivel alocar
		configASSERT(0);
		// TRATAR MELHOR ESSE ERRO
	}


//	SPI3->CR1 |= 0x1000;

	for(;;)
	{
		/* Fica suspensa até o momento que for chamada */
		vTaskSuspend(xHandleImage);

		// ### PERIGO, implementar um timer pq se travar aqui, a task triagem não pode ficar off pra sempre ###############################
		vTaskSuspend(triagemTaskHandle);

		// Recebe o feedback do RPi via IT
		HAL_UART_Receive_IT(&huart1, &BufferUART, 1);

		// Reseta o código, para estar em um ponto seguro
		HAL_UART_Transmit_IT(&huart1, &msgRPiReset, 1);

		// Semaforo liberado via UART Callback
		if( xSemaphoreTake(xSemBinImgUART, 10) == pdTRUE )
		{
			// tratar os possiveis retornos
			switch (BufferUART) {
				case 0x55:
					// Resposta esperada, Faça alguma coisa (pode informar no feedback)
					break;

				default:
					// Resposta inesperada, Faça alguma coisa (pode informar no feedback)
					break;
			}
		}
		else
		{
			// Não recebeu o feedback no tempo, timeout
			// O programa está fechado, ou o RPi travou, possivel solução, aguardar ou reiniciar o RPi
			configASSERT(0);
		}


		HAL_UART_Receive_IT(&huart1, &BufferUART, 1);
		HAL_SPI_Receive_IT(&hspi3, metadata, 5);

		osDelay(100);	// esse delay tem q sair daqui, seria bom algo mais inteligente, como uma msg do RPi avisando q está pronto

		HAL_UART_Transmit_IT(&huart1, &msgRPiMetadata, 1);

		// Aguarda feedback do RPi
		if( xSemaphoreTake(xSemBinImgUART, 10) == pdTRUE )
		{
			switch (BufferUART)
			{
				case 0x01:
					// Resposta esperada, Faça alguma coisa (pode informar no feedback)
					// metadado deve ter chego tbm
					if( xSemaphoreTake(xSemBinImgSPI, 10) == pdTRUE )
					{
						// Testa pra ver se o metadado é minimamente coerente
						for(int i = 0; i < 3; i++)
						{
							// Verifica se o primeiro byte é 'M' e o valor recebido é maior que 0
							//   Teoricamente esse 'if' NUNCA deveira ocorrer, por isso ele é uma camada puramente de segurança
							if( (metadata[0] != 'M') || ((metadata[1] << 24 | metadata[2] << 16 | metadata[3] << 8 | metadata[4]) == 0) )
							{
								// metadado zuado, pede pra mandar novamente

								// Aguarda 50ms pro RPi "pensar" (esse tempo é só por segurança, nada mt especifico)
								osDelay(50);

								// Solicita repetição da mensagem
								HAL_SPI_Receive_IT(&hspi3, metadata, 5);
								HAL_UART_Transmit_IT(&huart1, &msgRPiRepeat, 1);

								// Não estou verificando a resposta do UART (como feito na etapa anterior), corrigir isso mais pra frente

								if( xSemaphoreTake(xSemBinImgSPI, 20) == pdFALSE )		// ###### Precisa melhorar essa lógica, esse timeout impede as 3 tentativas ######
								{
									// Não chegou nada por SPI
									configASSERT(0);	// Timeout, reinicia essa task
									// Implementar lógica para reiniciar a task e informar via feedback o problema
								}

								if(i == 2)	// ######## Esse 'if' aqui vai gerar confusão se der certo na 3a tentativa, corrigir isso #######
								{
									// Mesmo apos 3 tentativas, não obteve sucesso
									configASSERT(0);	// Erro, não foi possivel receber os metadados sem erros, reinicia essa task
									// Implementar lógica para reiniciar a task e informar via feedback o problema
								}
							}
							else
							{
								// Aparentemente os metadados fazem algum sentido com o que se espera que eles sejam
								// pode sair do 'loop for'
								break;
							}
						} // end loop 'for' "teste de metadados"

						// Metadado recebido, (pode informar no feedback)
						// Transmitir para a GS os metadados

						/* Tenta por até 50 Ticks solicitar o Mutex SPI */
						if( xSemaphoreTake( xMutexSpiRadio, ( TickType_t ) 50 ) == pdTRUE )
						{
							SX_SetStandby(0x00);

							uint8_t *payloadMeta;
							payloadMeta = pvPortMalloc(7);

							// HEADER
							*(payloadMeta + 0) = 0x30;			// Codigo
							*(payloadMeta + 1) = 'M';			// Tipo | ID

							// PAYLOAD
							*(payloadMeta + 2) = metadata[1];
							*(payloadMeta + 3) = metadata[2];
							*(payloadMeta + 4) = metadata[3];
							*(payloadMeta + 5) = metadata[4];

							*(payloadMeta + 6) = '\n';				// ################################## remover esse

							/* Define os parametros do packet */
							SX_SetPacketParamsLoRa(0x08, 0x00, 7, 0x01, 0x00);

							/* Configura interrupção TX Done */
							SX_SetDioIrqParams(0x01, 0x01, 0, 0);

							SX_SetBufferBaseAddress(0x7F, 0x00);

							/* Escreve uma mensagem de teste no buffer */
							SX_WriteBuffer(0x7F, (uint8_t *) payloadMeta, 7);

							/* Transmite a mensagem que estava no buffer */
							SX_TX(0);

							/* Libera Mutex do SPI1 */
							xSemaphoreGive( xMutexSpiRadio );

							// Parametro para o feedback
							image.parameter[0] = 0x31;

							// libera memória alocada
							vPortFree(payloadMeta);

							// Em breve irá ocorrer uma interrupção TxDone e liberar o semaforo para esta task (essa interrupção vai ser gerada graças a transmissão acima)
							vTaskResume(triagemTaskHandle);

							// Se a GS retornar um ACK o código prossegue
							// Aguarda pela resposta da GS
							if( xSemaphoreTake(xSemBinRadioImage, portMAX_DELAY) == pdTRUE )		// AQUI PODE SER UM 'DO WHILE' e uma Queue, para informar se é necessário retransmitir os metadados
							{
								// Informa que recebeu o feedback positivo da GS
								image.parameter[0] = 0x32;

								// Agora que temos um ACK da GS, sabemos que ela conhece o tamanho total do arquivo
								//   proxima etapa é começar o envio

								// ######  Talvez seja necessário colocar um osDelay aqui se o feedback for muito rápido e não de tempo do RPi acompanhar kk

								// ### PERIGO, implementar um timer pq se travar aqui, a task triagem não pode ficar off pra sempre #######################################################
								vTaskSuspend(triagemTaskHandle);

								// Transmite feedbacks, e libera o semaforo xSemBinRadio para a etapa seguinte
								vTaskResume(xHandleTransmitter);

								// Prepara SPI
								HAL_SPI_Receive_IT(&hspi3, pBufferSPI, 4056);

								// Prepara buffer UART para receber resposta
								HAL_UART_Receive_IT(&huart1, &BufferUART, 1);

								// Solicita inicio do envio dos primeiros 4056 bytes
								HAL_UART_Transmit_IT(&huart1, &msgRPiBegin, 1);

								// Semaforo liberado via UART Callback
								if( xSemaphoreTake(xSemBinImgUART, 10) == pdTRUE )
								{
									// trata os possiveis retornos
									switch (BufferUART)
									{
										case 0x02:
											// Resposta esperada

											// Aguarda pelo recebimento dos 4056 bytes
											if( xSemaphoreTake(xSemBinImgSPI, 1000) == pdTRUE )
											{
												// Pacote recebido via SPI

												// Separa os Headers do payload
												ID = *(pBufferSPI + 0) << 8 | *(pBufferSPI + 1);
												size = *(pBufferSPI + 2) << 8 | *(pBufferSPI + 3);
												CRC32 = *(pBufferSPI + 4052) << 24 | *(pBufferSPI + 4053) << 16 | *(pBufferSPI + 4054) << 8 | *(pBufferSPI + 4055);

												// Verificar CRC do pacote, se falhar, solicite um reenvio


												// CRC OK, pode dar inicio a transmissão
												// Iniciar transmissão das 16 partes

												// HEADER
												// Reaproveita o espaço já alocado pra não precisar alocar mais um pedaço
												*(pBufferSPI + 2) = 0x30;	// Codigo
												*(pBufferSPI + 3) = 0x4F;	// Tipo | ID

												// Este loop for envia um pacote recebido do RPi, normalmente repete 16x
												for(int i = 0; i < size/253; i++)
												{
													// PAYLOAD

													// Verifica se o rádio está disponivel
													if (xSemaphoreTake(xSemBinRadio, 1000) == pdTRUE)
													{
														// Tenta solicitar o SPI1 para acessar o módulo, ele deve conseguir isso sempre caso esteja td bem com a lógica
														if( xSemaphoreTake( xMutexSpiRadio, ( TickType_t ) 50 ) == pdTRUE )
														{
															SX_SetStandby(0x00);

															/* Define os parametros do packet */
															SX_SetPacketParamsLoRa(0x08, 0x00, 255, 0x01, 0x00);

															/* Configura interrupção TX Done */
															SX_SetDioIrqParams(0x01, 0x01, 0, 0);

															// Define o tamanho maximo de buffer para TX
															SX_SetBufferBaseAddress(0x00, 0x00);

															/* Escreve uma mensagem no buffer */
															SX_WriteBuffer(0x00, (pBufferSPI + 2), 2);				// HEADER
															SX_WriteBuffer(0x02, (pBufferSPI + 4 + (i*253)), 253);	// PAYLOAD

															/* Transmite a mensagem que armazenada no buffer */
															SX_TX(0);

															/* Libera Mutex do SPI1 */
															xSemaphoreGive( xMutexSpiRadio );

														} // end 'if' MutexRadio
														else
														{
															// Por algum motivo não foi possivel adquirir o Mutex SPI
															configASSERT(0);
															// criar uma maneira de tratar a falha ou identificar o problema
														}

														// Subtrai 1 do ID
														*(pBufferSPI + 3) = *(pBufferSPI + 3) - 1;

													} // end 'if' SemBinRadio
													else
													{
														// Timeout, não houve TxDone para a transmissão dos 255 bytes
														// pode ser que o rádio esteja transmitindo ainda
														configASSERT(0);
													}

												} // end loop 'for' transmissão de pacotes
												  //  tudo que estava em pBufferSPI foi transmitido


												// PRECISA VERIFICAR SE HOUVE ALGUMA FALHA DE RECEBIMENTO NA GS
												// Aqui pode enviar um parametro de feedback dizendo que finalizou a transmissão dos pacotes e aguarda resposta da estação se chegou tudo OK
												//  ou se é necessário retransmitir alguma parte

												// Aguarda chegar msg numa queue

												image.parameter[1] = 'F';
												if(xSemaphoreTake(xSemBinRadio, 20000) == pdTRUE)
												{
													vTaskResume(xHandleTransmitter);
													vTaskResume(triagemTaskHandle);

													// Aguarda o feedback informando se houve erros de recepção e se é necessário retransmitir alguem
													if(xQueueReceive(xQueueRepeatMsg, RepeatMsg, 35000) == pdTRUE)
													{
														// ####################
														vTaskSuspend(triagemTaskHandle);

														// verifica se tem mais mensagens
														while(uxQueueMessagesWaiting(xQueueRepeatMsg) > 0)
														{
															// enquanto tiver mensagens, receba
															xQueueReceive(xQueueRepeatMsg, RepeatMsg, 0);
														}

														/* Comentario para continuação:
//	##################################					 * acabou a bateria kk, CONTINUE DAQUI
														 *
														 * Verifique se RepeatMsg está recebendo a quantidade de erros e os pacotes errados, em seguida fazer lógica para
														 *   separar os bytes de 4 em 4 bits e reenviar os errados */
//
//														for(int i = 0; i < RepeatMsg[0]; i++)
//														{
//															//SX_WriteBuffer(0x02, (pBufferSPI + 4 + (J*253)), 253);	// PAYLOAD
//															// onde J é os que chegarem errado
//														}

													}
													else
													{
														// timeout, nenhum feedback retornou
														configASSERT(0);
													}

												}
												else
												{
													// timeout
													configASSERT(0);
												}


											} // end 'if' SPI packet 4056 bytes
											else
											{
												// Timeout, pacote de 4056 bytes enviados por SPI pelo RPi não chegou a tempo
												// Feedback UART recebido com sucesso, mas retorno dos dados por SPI FALHOU
												// possivel causa: o RPi deu uma travada
												configASSERT(0);
											}

											break;

										default:
											// Resposta inesperada
											break;

									} // end 'switch case', feedback UART recebido

								} // end 'if' feedback UART RPi 4056 bytes
								else
								{
									// Timeout, Não recebeu o feedback no tempo
									// por algum motivo o RPi enviou os metadados anteriormente, mas agora misteriosamente ele não respondeu a solicitação de envio dos primeiros 4056 bytes

									// O RPi pode ter travado, possiveis soluções: aguardar um tempo ou reiniciar o programa ou reiniciar o RPi
									configASSERT(0);
								}
							} // end 'if' feedback GS
							else
							{
								// Timeout, a GS não retornou uma resposta dentro do tempo
								// Resposta relativa ao recebimento dos metadados
								configASSERT(0);
							}

						} // end 'if' radioMutexSPI
						else
						{
							// Por algum motivo não foi possivel adquirir o Mutex SPI
							configASSERT(0);
							// criar uma maneira de tratar a falha ou identificar o problema
						}

					} // end 'if' semBinSPI
					else
					{
						// Timeout, Metadata não chegou
						// Recebeu feedback correto por UART, mas não recebeu os metadados por SPI a tempo
						configASSERT(0);
					}
					break;

				default:
					// Resposta inesperada, tratar essa situação, transmissão uart foi corrompida ou estamos em outra parte do código
					// Recebeu algo em resposta a solicitação de metadados, mas não foi o que era esperado
					break;

			} // end 'switch case'

		} // end 'if' semBinUART
		else
		{
			// Não recebeu o feedback UART no tempo, timeout
			// A solicitação de metadados não foi atendida
			configASSERT(0);
		}


		vTaskResume(triagemTaskHandle);





















//		// -------------------------------------------------------------------
//		/* GS: Solicita Imagem, aí ele libera essa task
//		 * STM: Envia comando pro RPi enviar os metadados da foto q ele tirou
//		 * RPI: envia os metadados
//		 * STM: Verifica se são minimamente coerentes com o que se esperaria que fosse
//		 * STM: Envia esses metadados para GS
//		 * GS: Recebe eles e transmite ACK dizendo que recebeu
//		 * STM: Recebe ACK e continua para a etapa de envio do arquivo
//		 * */
//
//		// *************************#######################
//		// ASEGURE QUE A PORRA DO RPI TA DO JEITO QUE TEM QUE ESTAR, fazer um esquema de teste por exemplo, pra testar se ele ta engatilhado
//
//
//
//		while( (image.parameter[1] & 0xC0) != 0)		// enquanto não tiver a interrupção do SPI pra receber os metadados, ele fica aqui
//		{
//			image.parameter[1] = 0x00;
//			uint8_t feedbackPi = 0;
//
//			// Prepara SPI para metadados da imagem
//			image.parameter[1] = (image.parameter[1] & 0xCF) | (HAL_SPI_Receive_IT(&hspi3, metadata, 5)) << 4;
//
//			// STM diz: RPi me envie o metadado
//			image.parameter[1] = (image.parameter[1] & 0xF3) | (HAL_UART_Transmit(&huart1, &msgMetaData, 1, 100) << 2);
//
//			xSemaphoreTake(xSemBinRadioImage, portMAX_DELAY);
//		}
//
//		// Verifica se o metadado é minimamente coerente com o que é esperado receber
//		if( (metadata[0] != 'M') || ((metadata[1] << 24 | metadata[2] << 16 | metadata[3] << 8 | metadata[4]) == 0) )
//		{
//		//	metadado zuado, tem que mandar dnv
//			configASSERT(0);
//		}
//		// tem que implementar logica de transmitir o metadado e verificar se foi recebido
//		vTaskResume(xHandleTransmitter);
//		vTaskSuspend(xHandleImage);		// só pode avançar se receber um feedback positivo, caso contrario repita
//
//		RF_ID = (metadata[1] << 24 | metadata[2] << 16 | metadata[3] << 8 | metadata[4]) / 253;
//
//
//		// -------------------------------------------------------------------
//
//		// Prepara SPI para receber os primeiros 2024 bytes
//		HAL_SPI_Receive_IT(&hspi3, bufferSPI, 2032);
//
//		while(image.parameter[0] != 0x31)
//		{
//			image.parameter[1] = 'W';
//			HAL_UART_Transmit(&huart1, &msgRPiBegin, 1, HAL_MAX_DELAY);
//			vTaskSuspend(xHandleImage);
//		}
//
//		// Callback de SPI tira do while acima
//		// A partir daqui, a interrupção aconteceu e os 2032 bytes foram recebidos
//
//		ID = bufferSPI[0] << 8 | bufferSPI[1];
//		size = bufferSPI[2] << 8 | bufferSPI[3];
//		CRC32 = bufferSPI[2028] << 24 | bufferSPI[2029] << 16 | bufferSPI[2030] << 8 | bufferSPI[2031];
//
//		/* CRIAR ETAPA PARA VERIFICAR O CRC */
//
//		image.parameter[1] = 'T';
//
//		// Este timer assegura que a task Triagem não va ficar travada pra sempre caso essa função fique aqui por muito tempo
//		// Por exemplo, se eu ficar preso acidentalmente em um loop aqui, o timer gera uma interrupção que aborta esta tarefa
//		//   e retoma o controle para task Triagem, isso é uma etapa de segurança.
//		if( xTimerStart( xTimerTriagemWatchdog, 0 ) != pdPASS )
//			configASSERT(0);
//
//		// A partir de agora, o radio não entrará em RX MODE
//		vTaskSuspend(triagemTaskHandle);
//
//
//		// Este loop for envia um pacote recebido do RPi, normalmente repete 8x
//		for(int i = 0; i < size/253; i++)
//		{
//			// Primeiros 2 bytes da mensagem a ser transmitida
//			headerPayload[0] = RF_H | ((RF_ID & 0xF00) >> 12);
//			headerPayload[1] = RF_ID & 0xFF;
//
//			// Tenta solicitar o SPI1 para acessar o módulo, ele deve conseguir isso sempre caso esteja td bem com a lógica
//			if( xSemaphoreTake( xMutexSpiRadio, ( TickType_t ) 50 ) == pdTRUE )
//			{
//				SX_SetStandby(0x00);
//
//				/* Define os parametros do packet */
//				SX_SetPacketParamsLoRa(0x08, 0x00, 255, 0x01, 0x00);
//
//				/* Configura interrupção TX Done */
//				SX_SetDioIrqParams(0x01, 0x01, 0, 0);
//
//				// Define o tamanho maximo de buffer para TX
//				SX_SetBufferBaseAddress(0x00, 0x00);
//
//				/* Escreve uma mensagem no buffer */
//				SX_WriteBuffer(0x00, headerPayload, 2);					// primeira parte, header
//				SX_WriteBuffer(0x02, &bufferSPI[4 + (i*253)], 253);		// segunda parte, payload
//
//				/* Transmite a mensagem que estava no buffer */
//				SX_TX(0);
//
//				/* Libera Mutex do SPI1 */
//				xSemaphoreGive( xMutexSpiRadio );
//			}
//			else
//			{
//				// Por algum motivo não foi possivel adquirir o Mutex SPI
//				configASSERT(0);
//				// criar uma maneira de tratar a falha ou identificar o problema
//			}
//
//			if( xSemaphoreTake(xSemBinRadioImage, ( TickType_t ) 1000 ) == pdFALSE )			// ############ Criar metodo mais inteligente aqui pra gerar esse intervalo
//			{
//				// a interrupção TxDone não ocorreu dentro do intervalo de ticks
//				configASSERT(0);
//				// criar uma maneira de tratar a falha ou identificar o problema
//			}
//
//			RF_ID--;
//
//		}
//
//		vTaskResume(xHandleTransmitter);	// beacon telemetria   ################## Implementar um outro metodo a mais que "pergunte" se é necessário enviar novamente alguma parte que foi corrompida
//
//		image.parameter[0] = 0x30;
//		image.parameter[1] = 'S';
//
//		// Libera semaforo da task Triagem, se isso não for feito, ela pode acusar falha de TxDone quando retomar
//		xSemaphoreGive(xSemBinRadio);
//		vTaskResume(triagemTaskHandle);
//
//		// Desativa timer
//		if( xTimerStop( xTimerTriagemWatchdog, 0 ) != pdPASS )
//			configASSERT(0);
//
//
//		/* Use the handle to obtain further information about the task. */		// ###### Está aqui só pra teste, pra ver quanta memoria a task está consumindo
//		vTaskGetInfo( /* The handle of the task being queried. */
//			  xHandleImage,
//			  /* The TaskStatus_t structure to complete with information
//			  on xTask. */
//			  &xTaskDetails,
//			  /* Include the stack high water mark value in the
//			  TaskStatus_t structure. */
//			  pdTRUE,
//			  /* Include the task state in the TaskStatus_t structure. */
//			  eInvalid );
//		__NOP();

	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_TriagemTask */
/**
  * @brief  Function implementing the triagemTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TriagemTask */
void TriagemTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	/* -=-=-=-=-=-=-=-=- TASK 0 - Triagem -=-=-=-=-=-=-=-=-	*
	 * 														*
	 * Esta task está sempre ativa, todas as mensagem que	*
	 *  chegam irão obrigatóriamente passar inicialmente 	*
	 *  por ela.											*
	 *  													*
	 * Quando uma nova mensagem chega, ela contem alguma 	*
	 *  informação, como por exemplo: um comando, seguido	*
	 *  de um parâmetro. Esse comando ira informar para		*
	 *  qual task o parâmetro deve ser encaminhado			*
	 * 														*
	 * - - - - - - - - - - - - - - - - - - - - - - - - - - 	*
	 * 														*
	 *  > FUNCOES DESTA TASK <								*
	 *  													*
	 * > Receber via interrupção as mensagens que chegam;	*
	 * > Identificar qual é a task processará o parametro	*
	 *    recebido (switch case);							*
	 * > Enviar um ACK informando a recepção;				*
	 * -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= *
	 */

	/* xSemBinRadio: Semaforo Binario responsável por permitir que o radio entre no modo RX	*/
	xSemBinRadio = xSemaphoreCreateBinary();

	/* xSemBinRx: Semaforo Binario que informa via interrupção que existe uma nova mensagem no buffer do rádio */
	xSemBinRX = xSemaphoreCreateBinary();

	// xQueueRepeatMsg: As mensagens que chegarem corrompidas para GS, serão informadas atravez dessa Queue
	xQueueRepeatMsg = xQueueCreate(17, sizeof(uint8_t));

	/* Verifica se foi possivel criar os dois semaforos */
	if( (xSemBinRadio == NULL) || (xSemBinRX == NULL) )
	{
		//  # IMPLEMENTAR TRATAMENTO DE ERRO #

		/* There was insufficient FreeRTOS heap available for the semaphore to
		be created successfully. */
		configASSERT(0);
	}

	uint16_t periodo = 2000;	// Periodo entre cada ciclo Tx Rx
	uint8_t status = 0, PayloadLengthRx = 0, RxStartBufferPointer = 0;
	uint8_t *payload = 0;		// Ponteiro para o pacote recebido após cara RxDone

	uint8_t BlockFeedback = 0;

	/* Suspende a esta Task momentaneamente. Caso o rádio não enteja conectado, o primeiro if nunca ocorreria */
	vTaskSuspend(triagemTaskHandle);		// Liberado pela task de transmissão

  /* Infinite loop */
	for(;;)
	{
		/* Aguarda por até 3x o tempo necessário um ciclo de transmissão e recepção,
		 * isso foi implementado para que se por algum motivo a interrupção TxDone falhar,
		 * o codigo não fique travado esperando para sempre */

		/* Semaforo liberado via interrupção TxDone */
		if( xSemaphoreTake(xSemBinRadio, ( TickType_t ) periodo*4 ) == pdTRUE )
		{
			/* Tenta por até 50 Ticks solicitar o Mutex SPI */
			if( xSemaphoreTake( xMutexSpiRadio, ( TickType_t ) 50 ) == pdTRUE )
			{
				/* Configura interrupção RX Done */
				SX_SetDioIrqParams(0x02, 0, 0x02, 0);

				/* Coloca o rádio em RX Mode*/
				SX_RX(0);

				/* Libera Mutex do SPI1 */
				xSemaphoreGive(xMutexSpiRadio);

				/* Aguarda por um periodo o recebimento de alguma mensagem */
				/* Semaforo liberado via interrupção RxDone */
				if( xSemaphoreTake(xSemBinRX, ( TickType_t ) periodo ) == pdTRUE)
				{
					/* Tenta por até 50 Ticks solicitar o Mutex SPI */
					if( xSemaphoreTake( xMutexSpiRadio, ( TickType_t ) 50 ) == pdTRUE )
					{
						SX_GetRxBufferStatus(&status, &PayloadLengthRx, &RxStartBufferPointer);
						SX_ReadBuffer(&payload, RxStartBufferPointer, PayloadLengthRx - RxStartBufferPointer);

						/* Libera Mutex do SPI1 */
						xSemaphoreGive(xMutexSpiRadio);

						/* De acordo com o que for recebido, este switch case faz a triagem para a task correspondente */
						switch (*payload) {
							case 0x50:
								vTaskResume(xHandleDeploy);
								break;

							case 0x46:
								// Libero a task Imagem e bloqueio o feedback por 1 ciclo, quando Task Image for acionada, ela suspende Triagem
								// Assim que Task Imagem estiver no jeito pra transmitir o metadado, ela libera Triagem (que está aguardando o semaforo agr)
								//   e o semaforo TxDone ocorre quando o metadado é transmitido, após isso, o ciclo normal se repete
								vTaskResume(xHandleImage);
								BlockFeedback = 1;
								break;

							case 0x66:
								xSemaphoreGive(xSemBinRadioImage);	// A Ground Station informa que recebeu os metadados corretamente
								BlockFeedback = 1;
								break;

							case 0x69:
								// Aqui recebe os pacotinhos corrompidos e informa via queue para Imagem se houve ou não
								// payload[0]: ID = 0x69
								// payload[1]: quantos : 0 ~ F
								// payload[2~9]: 0xFE 0xDC 0xBA 0x98 0x76 0x54 0x32 0x10

								if( *(payload + 1) <= 16 )
								{
									for(int i = 0; i <= *(payload + 1); i++)	// sempre vai colocar a quantidade pelo menos, mesmo que seja 0
										xQueueSend(xQueueRepeatMsg, *(payload + 1 + i), 0);
								}

								BlockFeedback = 1;
								break;

							default:
								break;
						}
					}
					else
					{
						// Por algum motivo não foi possivel adquirir o Mutex SPI
						configASSERT(0);
						// criar uma maneira de informar que houve falha
					}
				}
			}
			else
			{
				// Por algum motivo não foi possivel adquirir o Mutex SPI
				configASSERT(0);
				// criar uma maneira de informar que houve falha
			}
		}
		else
		{
			// A interrupção TxDone falhou por algum motivo
			configASSERT(0);

			// criar uma maneira de informar que houve falha
		}


		// bloqueia o envio de feedbacks por um determinado numero de vezes caso seja necessário
		// Task imagem que necessita dessa lógica
		if(uxQueueMessagesWaiting(xQueueBlockFeedback) > 0)
		{
			xQueueReceive(xQueueBlockFeedback, &BlockFeedback, ( TickType_t ) 0);
		}

		if (BlockFeedback == 0)
		{
			/* Libera Task de transmissão */
			vTaskResume(xHandleTransmitter);
		}
		else
		{
			BlockFeedback--;
		}

	}


  /* USER CODE END 5 */
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
