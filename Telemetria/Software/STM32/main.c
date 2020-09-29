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

QueueHandle_t xQueueRadio;

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
                  256,      				/* Stack size in words, not bytes. */
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
                    550,      				/* Stack size in words, not bytes. */
                    ( void * ) 1,    		/* Parameter passed into the task. */
  				  ( ( UBaseType_t ) 3U ),	/* Priority at which the task is created. */
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
  osThreadDef(triagemTask, TriagemTask, osPriorityNormal, 0, 128);
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

			if(image.parameter[0] != 0x31)
			{
				/* Este semaforo indica que a transmissão de dados finalizou, e agora o
				 *   modo de recepção pode começar */
				xSemaphoreGiveFromISR(xSemBinRadio, &xHigherPriorityTaskWoken);
			}
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
				SX_SetPacketParamsLoRa(0x08, 0x00, sizeof(payload), 0x01, 0x00);

				/* Configura interrupção TX Done */
				SX_SetDioIrqParams(0x01, 0x01, 0, 0);

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
	image.parameter[0] = 0x31;

	BaseType_t xYieldRequired;

	 // Resume the suspended task.
	 xYieldRequired = xTaskResumeFromISR( xHandleImage );

	 if( xYieldRequired == pdTRUE )
	 {
		 // We should switch context so the ISR returns to a different task.
		 // NOTE:  How this is done depends on the port you are using.  Check
		 // the documentation and examples for your port.
		 portYIELD_FROM_ISR(xYieldRequired);
	 }
}

void vTimerCallback( TimerHandle_t xTimer )
{
	// Se der merda, o timer não será parado, então isso garante que volte pra um estado conhecido

	image.parameter[0] = 0x30;
	image.parameter[1] = 'F';
	xSemaphoreGive(xSemBinRadio);
	vTaskResume(triagemTaskHandle);
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

	feedback *pImage = &image;

	uint8_t msgRPiBegin[2] = {0xFF, 0xAA};
	uint8_t msgRPiNext[2] = {0xFF, 0xCC};
	uint8_t msgRPiRepeat[2] = {0xFF, 0x00};

	uint8_t bufferSPI[2032];

	image.ID_Task[0] = 'I'; 	image.ID_Task[1] = 'M'; 	image.ID_Task[2] = 'G';
	image.parameter[0] = 0x30; 	image.parameter[1] = 0x30;

	if( xQueueSend(xQueueRadio, (void *) &pImage, ( TickType_t ) 100) != pdPASS)
	{
		/* Não conseguiu colocar ponteiro na queue*/
		configASSERT(0);
	}

//	SPI3->CR1 |= 0x1000;

	for(;;)							// ########################### IMPLEMENTAR MAIS ETAPAS DE SEGURANÇA
	{
		/* Fica suspensa até o momento que for chamada */
		vTaskSuspend(xHandleImage);

		HAL_SPI_Receive_IT(&hspi3, bufferSPI, 2032);

		while(image.parameter[0] != 0x31)
		{
			image.parameter[1] = 'W';
			HAL_UART_Transmit(&huart1, msgRPiBegin, 2, HAL_MAX_DELAY);
			vTaskSuspend(xHandleImage);
		}

		image.parameter[1] = 'T';

		if( xTimerStart( xTimerTriagemWatchdog, 0 ) != pdPASS )			// ################ O TIMER TA BUGANDO O DEPLOY, deve ser falta de memoria
			configASSERT(0);

		vTaskSuspend(triagemTaskHandle);


		for(int i = 0; i < 5; i++)
		{
			osDelay(5000);		// simula a transmissão
			vTaskResume(xHandleTransmitter);	// beacon
		}

		image.parameter[0] = 0x30;
		image.parameter[1] = 'S';
		xSemaphoreGive(xSemBinRadio);
		vTaskResume(triagemTaskHandle);

		if( xTimerStop( xTimerTriagemWatchdog, 0 ) != pdPASS )
			configASSERT(0);


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

	/* Suspende a esta Task momentaneamente. Caso o rádio não enteja conectado, o primeiro if nunca ocorreria */
	vTaskSuspend(triagemTaskHandle);		// Liberado pela task de transmissão

  /* Infinite loop */
	for(;;)
	{
		/* Aguarda por até 3x o tempo necessário um ciclo de transmissão e recepção,
		 * isso foi implementado para que se por algum motivo a interrupção TxDone falhar,
		 * o codigo não fique travado esperando para sempre */

		/* Semaforo liberado via interrupção TxDone */
		if( xSemaphoreTake(xSemBinRadio, ( TickType_t ) periodo*3 ) == pdTRUE )
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
								vTaskResume(xHandleImage);
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

		/* Libera Task de transmissão */
		vTaskResume(xHandleTransmitter);

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
