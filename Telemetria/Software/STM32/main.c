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
#include "message_buffer.h"
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

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

osThreadId standbyTaskHandle;
/* USER CODE BEGIN PV */

// ###  FreeRTOS  ###

TaskHandle_t xHandleRadio = NULL;
TaskHandle_t xHandleImage = NULL;
TaskHandle_t xHandleControl = NULL;
TaskHandle_t xHandleDeploy = NULL;
TaskHandle_t xHandleMemory = NULL;
TaskHandle_t xHandleSensor = NULL;

SemaphoreHandle_t xSemBinUART = NULL;
SemaphoreHandle_t xSemBinSPI = NULL;
SemaphoreHandle_t xSemBinSendImage = NULL;

TimerHandle_t xTimerRTOS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
void StandbyTask(void const * argument);

/* USER CODE BEGIN PFP */

// ###  FreeRTOS  ###

void vTaskRadio( void * pvParameters );
void vTaskImage( void * pvParameters );
void vTaskControl( void * pvParameters );
void vTaskDeploy( void * pvParameters );
void vTaskMemory( void * pvParameters );
void vTaskSensor( void * pvParameters );

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
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  // -----------------------------------------------------------------------------
  // -----------------------------------------------------------------------------

  BaseType_t xReturned;


  // -----------------------------------------------------------------------------
  /* Task Transmitter, equivalente a task 1 do modelo,
   *  esta task configura o radio e transmite os dados
   *  periodicamente */

  xReturned = xTaskCreate(
                  vTaskRadio,       		/* Function that implements the task. */
                  "Radio",    				/* Text name for the task. */
                  256,      				/* Stack size in words, not bytes. */
                  ( void * ) 1,    			/* Parameter passed into the task. */
				  ( ( UBaseType_t ) 1U ),	/* Priority at which the task is created. */
                  &xHandleRadio );    /* Used to pass out the created task's handle. */

  if( xReturned != pdPASS )
  {
	  //  # IMPLEMENTAR TRATAMENTO DE ERRO #
	  configASSERT(0);
  }
  // -----------------------------------------------------------------------------
    /* Task Image, equivalente a task 2 do modelo,
     *  esta task faz a aquisição de um pedaço da
     *  imagem capturada pelo Raspberry */

    xReturned = xTaskCreate(				/* Create the task, storing the handle. */
                    vTaskImage,       		/* Function that implements the task. */
                    "Image",    			/* Text name for the task. */
                    128,      				/* Stack size in words, not bytes. */
                    ( void * ) 1,    		/* Parameter passed into the task. */
  				  ( ( UBaseType_t ) 2U ),	/* Priority at which the task is created. */
                    &xHandleImage );   /* Used to pass out the created task's handle. */

    if( xReturned != pdPASS )
    {
  	  //  # IMPLEMENTAR TRATAMENTO DE ERRO #
  	  configASSERT(0);
    }
    // -----------------------------------------------------------------------------
    xReturned = xTaskCreate(
                    vTaskControl,       	/* Function that implements the task. */
                    "Control",    				/* Text name for the task. */
                    64,      				/* Stack size in words, not bytes. */
                    ( void * ) 1,    			/* Parameter passed into the task. */
  				  ( ( UBaseType_t ) 3U ),	/* Priority at which the task is created. */
                    &xHandleControl );    /* Used to pass out the created task's handle. */

    if( xReturned != pdPASS )
    {
  	  //  # IMPLEMENTAR TRATAMENTO DE ERRO #
  	  configASSERT(0);
    }
    // -----------------------------------------------------------------------------
    /* Task Deploy, equivalente a task 4 do modelo,
     *  esta task realiza o deploy da antena e em
     *  seguida retorna um feedback para task 1   */

    xReturned = xTaskCreate(
                    vTaskDeploy,       		/* Function that implements the task. */
                    "Deploy",    				/* Text name for the task. */
                    64,      					/* Stack size in words, not bytes. */
                    ( void * ) 1,    			/* Parameter passed into the task. */
  				  ( ( UBaseType_t ) 4U ),	/* Priority at which the task is created. */
                    &xHandleDeploy );    		/* Used to pass out the created task's handle. */

      if( xReturned != pdPASS )
      {
      	//  # IMPLEMENTAR TRATAMENTO DE ERRO #
      	configASSERT(0);
      }
      // -----------------------------------------------------------------------------
      /* */

      xReturned = xTaskCreate(
                      vTaskMemory,       		/* Function that implements the task. */
                      "Memory",    				/* Text name for the task. */
                      64,      				/* Stack size in words, not bytes. */
                      ( void * ) 1,    			/* Parameter passed into the task. */
    				  ( ( UBaseType_t ) 2U ),	/* Priority at which the task is created. */
                      &xHandleMemory );    /* Used to pass out the created task's handle. */

      if( xReturned != pdPASS )
      {
    	  //  # IMPLEMENTAR TRATAMENTO DE ERRO #
    	  configASSERT(0);
      }
      // -----------------------------------------------------------------------------
      /* Task Transmitter, equivalente a task 1 do modelo,
       *  esta task configura o radio e transmite os dados
       *  periodicamente */

      xReturned = xTaskCreate(
                      vTaskSensor,       		/* Function that implements the task. */
                      "Sensor",    				/* Text name for the task. */
                      64,      				/* Stack size in words, not bytes. */
                      ( void * ) 1,    			/* Parameter passed into the task. */
    				  ( ( UBaseType_t ) 4U ),	/* Priority at which the task is created. */
                      &xHandleSensor );    /* Used to pass out the created task's handle. */

      if( xReturned != pdPASS )
      {
    	  //  # IMPLEMENTAR TRATAMENTO DE ERRO #
    	  configASSERT(0);
      }
      // -----------------------------------------------------------------------------
//      xTimerRTOS = xTimerCreate
//						   (
//							 "TimerRTOS",	/* Just a text name, not used by the RTOS kernel. */
//							 50000,			/* The timer period in ticks, must be greater than 0. */
//							 pdFALSE,		/* The timers will auto-reload themselves when they expire. */
//							 ( void * ) 0,	/* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
//							 vTimerCallback	/* Each timer calls the same callback when it expires. */
//						   );
//
//    if( xTimerRTOS == NULL )
//	{
//    	configASSERT(0);					/* The timer was not created. */
//	}
    // -----------------------------------------------------------------------------
    // -----------------------------------------------------------------------------

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
  /* definition and creation of standbyTask */


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
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 79764919;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_32B;
  hcrc.Init.InitValue = 0xFFFFFFFF;
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
  htim7.Init.Prescaler = 4;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 23;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim7, TIM_OPMODE_SINGLE) != HAL_OK)
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

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------


void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(xSemBinSPI, &xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(xSemBinUART, &xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* Struct feedback é a forma padronizada que as tasks vão informar um feedback para a GS */
typedef struct
{
	uint8_t ID_Task[2];					// ID ou TAG para associar a task
	volatile uint8_t parameter[3];		// Parametro a ser passado como feedback
} feedback;

feedback FbImg, FbCtrl, FbDpl, FbMmry, FbSensor, FbRadio;

// Status Task Deploy
#define FOLDED  		0x0
#define DEPLOYING  		0x1
#define NOT_DEPLOYED  	0x2
#define FULLY_DEPLOYED  0x3

// RPi Task Image
#define CONFIG		0x00
#define FINISH		0xFF
#define SUSPEND		0x55
#define PICTURE		0x0F
#define SEND		0xC3
#define CONTINUE	0xA5


uint8_t *pBufferSPI = NULL;


// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	/* Usado em caso de necessidade de mudança de contexto entre as tasks */
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	switch (GPIO_Pin) {
		case Fdb_Deploy_Pin:
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);										// Desativa elemento de aquecimento
			xTaskNotifyFromISR(xHandleDeploy, FULLY_DEPLOYED, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);		// Informa Task Deploy
			break;

		case DIO1_Pin:
			/* Interrupção TX DONE */
			SX_ClearIrqStatus(0x01);											// Limpa flags
			HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET);		// Desativa o amplificador de RF (TX) para poupar (MUITA) energia

			xTaskNotifyFromISR(xHandleRadio, 0x06, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);		// Informa Task Radio, via Notificacao
			break;

		case DIO2_Pin:
			/* Interrupção RX DONE */
			SX_ClearIrqStatus(0x02);											// Limpa flags
			HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_RESET);		// Desativa o amplificador de RF (RX) para poupar energia

			xTaskNotifyFromISR(xHandleRadio, 0x09, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);		// Informa Task Radio, via Notificacao
			break;
	}

	/* Macro que verifica necessidade de mudança de contexto entre as tasks */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------

void vTaskRadio( void * pvParameters )
{
	// Desenvolvimento
	UBaseType_t uxHighWaterMarkRadio;
	uxHighWaterMarkRadio = uxTaskGetStackHighWaterMark( NULL );

	// Feedback
	FbRadio.ID_Task[0] = 'R';	FbRadio.ID_Task[1] = 'D';
	FbRadio.parameter[0] = ' ';	FbRadio.parameter[1] = ' ';	FbRadio.parameter[2] = ' ';

	/* Array de ponteiros que armazena todos os endereços das variaveis de feedbacks */
	feedback *pArrayFeedback[] = {&FbDpl, &FbImg, &FbRadio, &FbCtrl, &FbMmry, &FbSensor};

	// State Machine
	enum {suspend, receive, transmit, screening};
	uint8_t state = receive;

	// Task Notify
	uint32_t radioFeedback = 0;

	// Radio RX
	uint8_t status, PayloadLengthRx, RxStartBufferPointer;
	uint8_t *pRadioBuffer = NULL;

	// Radio TX
	uint8_t payload[255];
	uint16_t contador = 0;
	for(int i = 0; i < sizeof(payload); i++)	payload[i] = 0x00;		// zera a memoria, mensagem de feedback
	uint8_t resto = 0, ultimo = 0;

	// Image
	_Bool flag_metadada = 0;
	uint8_t metadata[6], message_repeat[9];
	uint16_t ID, size;

	//-----------------------------------------------------------------------------------------------------------------

	// -=-=-=-=-=-=-=-=- TIMER_5 SETUP -=-=-=-=-=-=-=-=-
	// Este timer serve para dar tempo ao BUSY, especificado em datasheet, pag 52, Tsw.
	// Está configurado para operar com 480MHz no clock principal, 240MHz em APB1_Timer_clock

	// Timer configurado em 400n para clock de 240MHz, o resultado final considera o tempo do timer + o atraso do codigo
//	TIM7->PSC = 0x04;		// Clock prescaler
//	TIM7->ARR = 23;		// Auto-reload register

	TIM7->SR = 0x00;		// Clear Flag Interrupt
	TIM7->DIER |= 0x01;		// Update interrupt enable

	//-----------------------------------------------------------------------------------------------------------------

	/* -=-=-=-=-=-=-=-=- RADIO SETUP -=-=-=-=-=-=-=-=- */

	uint8_t LoRaSyncWord[2] = { 0x14, 0x24 };							// Sync Word Lora

	HAL_GPIO_WritePin(NRST_GPIO_Port, NRST_Pin, GPIO_PIN_SET);			// Habilita o rádio
	HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET);		// Desativa o amplificador de RF (TX) para poupar (MUITA) energia
	HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_RESET);		// Desativa o amplificador de RF (RX) para poupar energia

	osDelay(50);														// Aguarda um tempinho para estabilização do radio

	SX_SetStandby(0x00);												// Modo Standby para poder realizar as configurações iniciais

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

	/* Setup do rádio */					//  #### Para ficar mais intuitivo, utilizar #defines    ####
	SX_SetPacketType(0x01);
	SX_SetFrequency(915e6);
	SX_SetPaConfig(0x04, 0x07, 0x00);		// Afeta a potencia de saida	0x04 0x07 0x00 MAX
	SX_SetTxParam(4, 0x03);					// Afeta a potencia de saida	22 MAX
	SX_SetBufferBaseAddress(0x7F, 0x00);
	SX_SetModulationParamsLoRa(0x07, 0x04, 0x01, 0x00);
	SX_WriteRegister(0x0740, LoRaSyncWord, 2);
//----------------------------------------------------------------------------------------------------------------------------------
										/* -=-=-=-=-=-=-=-=- LOOP RADIO -=-=-=-=-=-=-=-=- */
	for(;;)
	{
		switch (state)
		{
			case suspend:
				if( xTaskNotifyWait(0x00, 0xFFFFFFFF, &radioFeedback, 2000) == pdTRUE)		// Notificado via interrupcao externa
				{
					if( 0x06 == (radioFeedback & 0x0F) )
					{	// Causado pela interrupção TxDone, o procedimento de envio finalizou
						state = receive;
					}
					else if( 0x09 == (radioFeedback & 0x0F) )
					{	// Causado pela interrupção RxDone, algo foi recebido
						state = screening;
					}
					else if( 0x0B == (radioFeedback & 0x0F) )
					{	// Aviso da Task Image informando que está pronta para transmitir
						HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_RESET);		// Desativa o amplificador de RF (RX) para poupar energia
						state = transmit;
					}
				}
				else
				{	// Timeout, o radio não gerou interrupção, transmite telemetria
					HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_RESET);			// Desativa o amplificador de RF (RX) para poupar energia
					state = transmit;

					uxHighWaterMarkRadio = uxTaskGetStackHighWaterMark( NULL );				// Remover isso daqui futuramente, está aqui para analise ##################
				}
				break;
//----------------------------------------------------------------------------------------------------------------------------------
			case receive:
				SX_SetDioIrqParams(0x02, 0, 0x02, 0);										// Configura interrupção RX Done
				SX_RX(0);																	// Coloca o rádio em RX Mode
				state = suspend;
				break;
//----------------------------------------------------------------------------------------------------------------------------------
			case transmit:					//		=-=-=-=-=  IMAGEM  =-=-=-=-=
				if(xSemaphoreTake(xSemBinSendImage, 0) == pdTRUE)							// Semaforo relacionado com o envio da imagem
				{
					if(flag_metadada)		//	    - - - --  Metadados  -- - - - 			// Flag para realizar aquisição do metadado uma única vez
					{
						flag_metadada = 0;													// clear flag

						metadata[0] = 0x50;					// HEADER						// Codigo de identificação
						metadata[1] = 0x4D;					// HEADER						// Tipo de dado
						metadata[2] = *(pBufferSPI + 4052);	// PAYLOAD
						metadata[3] = *(pBufferSPI + 4053);	// PAYLOAD
						metadata[4] = *(pBufferSPI + 4054);	// PAYLOAD
						metadata[5] = *(pBufferSPI + 4055);	// PAYLOAD

						SX_SetStandby(0x00);												// Inicia processo de transmissão
						SX_SetPacketParamsLoRa(0x08, 0x00, sizeof(metadata), 0x01, 0x00);	// Define parametros da transmissão
						SX_SetDioIrqParams(0x01, 0x01, 0, 0);								// Define interrupção
						SX_SetBufferBaseAddress(0x00, 0x00);								// Define endereços no FIFO do Rádio
						SX_WriteBuffer(0x00, metadata, sizeof(metadata));					// Escreve Buffer
						SX_TX(0);															// Transmite
					}
					else					//	      - - - --  Imagem  -- - - -
					{	// verdadeiro na primeira vez que entrar aqui, em caso de solicitacao de reenvio, isso sera falso
						if(!(message_repeat[0] & 0x80))
						{
							size = *(pBufferSPI + 2) << 8 | *(pBufferSPI + 3);				// 'size' recebe o tamanho do pacote, normalmente é 4048 e o ultimo costuma ser menor
							message_repeat[0] = 0x80 | (size/253);							// 'message_repeat' informa quantas mensagens são necessários transmitir (ou repetir em caso de solicitaçao de reenvio)
																							//     se for o primeiro envio, "repita todas" (equivale a enviar todas), se for uma solicitação de retransmissão, retransmite apenas a quantidade solicitada
							resto = size % 253;
							if(resto)														// Normalmente o ultimo pacote não tem tamanho multiplo de 253, então vai sobrar um 'resto' de bytes para ser transmitidos
							{
								ultimo = message_repeat[0] & 0x1F;							// o ultimo pacote vai ter um tratamento especial pra não ser transmitido lixo de memória, o tamanho dele vai ser menor que 253 (255) bytes
								message_repeat[0]++;										// soma 1 pra poder transmitir os bytes restantes
							}
							else
								ultimo = 255;												// só um numero grande pra exessao não ocorrer quando não deve

							ID = *(pBufferSPI + 0) << 8 | *(pBufferSPI + 1);				// ID decrescente que informa quantos pacotes de 4048 ainda faltam
							FbImg.parameter[0] = 'I'; FbImg.parameter[1] = 'D'; FbImg.parameter[2] = 0x30 | (0xF & ID);
						}

						*(pBufferSPI + 4058) = 0x50;	// Codigo de identificação			// HEADER		Reaproveita o espaço já alocado pra não precisar alocar mais um pedaço
//					- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
						for(int i = 0; i < (message_repeat[0] & 0x1F); i++)
						{
							if(i%2 == 0)													// HEADER		// Logica que separa 1 byte em 2 numeros
								*(pBufferSPI + 4059) = ( 0x40 | (message_repeat[1 + (i/2)] >> 4) );			// Os pacotes a serem transmitidos chegam agrupados de 2 em 2 para cada byte de 'message_repeat[1~9]'
							else																			//   essa lógica separa esses números para que possam ser transmitidos corretamente
								*(pBufferSPI + 4059) = 0x40 | (message_repeat[1 + (i/2)] & 0x0F);

							SX_SetStandby(0x00);											// Inicia procedimento de transmissão

							if( (*(pBufferSPI + 4059) & 0x0F) == ultimo)					// (*(pBufferSPI + 4059) & 0x0F) é o indice da transmissão, no dado transmitido corresponde ao 'n' em '0x4n'
								SX_SetPacketParamsLoRa(0x08, 0x00, resto+2, 0x01, 0x00);	// se for o ultimo pacote, e ele for menor que 253, essa lógica garante que não será transmitido lixo de momória
							else
								SX_SetPacketParamsLoRa(0x08, 0x00, 255, 0x01, 0x00);		// em casos normais, o pacote terá o tamanho máximo

							SX_SetDioIrqParams(0x01, 0x01, 0, 0);
							SX_SetBufferBaseAddress(0x00, 0x00);
							SX_WriteBuffer(0x00, (pBufferSPI + 4058), 2);					// HEADER

							if( (*(pBufferSPI + 4059) & 0x0F) == ultimo)
								SX_WriteBuffer(0x02, (pBufferSPI + 4 + ( ((*(pBufferSPI + 4059)) & 0x0F) * 253 )), resto);	// PAYLOAD, mesma lógica comentada acima
							else
								SX_WriteBuffer(0x02, (pBufferSPI + 4 + ( ((*(pBufferSPI + 4059)) & 0x0F) * 253 )), 253);	// PAYLOAD
							SX_TX(0);																						// Inicia transmissão

							if( (*(pBufferSPI + 4059) & 0x0F) != ((message_repeat[0] & 0x1F)-1) )	// Se não for o ultimo pacote a ser transmitido,
								xTaskNotifyWait(0x00, 0xFFFFFFFF, &radioFeedback, 5000);			//   aguarda pela interrupcao TxDone aqui mesmo, se não, a interrupção vai ser tradada normalmente no estado de suspensão
						}
					}
				}
				else						//		=-=-=-=-=  TELEMETRIA  =-=-=-=-=
				{
					sprintf( (char*)payload, "U%.3d|", contador);							// Mensagem que será enviada
					if(++contador >= 1000)													// Contador de envios
						contador = 0;

					for(int i = 0, j = 0; j < 3; i = i + 6, j++)							// Adiciona todos os feedbacks das outras Tasks na mensagem de telemetria
					{																		// j < 'x' , onde 'x' é a quantidade de feedbacks para serem transmitido
						payload[5+i] = pArrayFeedback[j]->ID_Task[0];						// A ordem de envio depende da ordem em que estão declarados no Array
						payload[6+i] = pArrayFeedback[j]->ID_Task[1];
						payload[7+i] = pArrayFeedback[j]->parameter[0];
						payload[8+i] = pArrayFeedback[j]->parameter[1];
						payload[9+i] = pArrayFeedback[j]->parameter[2];
						payload[10+i] = '|';
					}
					payload[39] = '\n';

					SX_SetStandby(0x00);													// Procedimento de envio
					SX_SetPacketParamsLoRa(0x08, 0x00, 40, 0x01, 0x00);
					SX_SetDioIrqParams(0x01, 0x01, 0, 0);
					SX_SetBufferBaseAddress(0x00, 0x00);
					SX_WriteBuffer(0x00, (uint8_t *) payload, 40);
					SX_TX(0);
				}
				state = suspend;															// Vai para o estado suspenso enquanto aguarda TxDone
				break;
//----------------------------------------------------------------------------------------------------------------------------------
			case screening:																	// Identifica qual atitude tomar, mediante a informacao que chegou
				SX_GetRxBufferStatus(&status, &PayloadLengthRx, &RxStartBufferPointer);		// Recebe a mensagem do buffer do Radio
				SX_ReadBuffer(&pRadioBuffer, RxStartBufferPointer, PayloadLengthRx - RxStartBufferPointer);

				if(*pRadioBuffer == 0x55)							// Serve para separar as mensagem espúrias, funciona como o 'nome' do receptor
				{
					state = transmit;								// caso o comando não necessite de um estado diferente, o proximo estado normalmente é o de transmissão, para enviar um feedback
					switch(*(pRadioBuffer+1))						// ID do comando
					{
						case 0x50:		// Deploy Antena
							FbRadio.parameter[0] = 'D';	FbRadio.parameter[1] = 'P';	FbRadio.parameter[2] = 'L';
							break;

						case 0x46:		// Procedimento para enviar imagem (metadado)
							FbRadio.parameter[0] = 'I';	FbRadio.parameter[1] = 'M';	FbRadio.parameter[2] = 'G';
							flag_metadada = 1;
							xTaskNotify(xHandleImage, SUSPEND, eSetValueWithOverwrite);
							vTaskResume(xHandleImage);
							state = suspend;
							break;

						case 0x47:		// Repetir o metadado
							FbRadio.parameter[0] = 'M';	FbRadio.parameter[1] = 'S';	FbRadio.parameter[2] = 'D';
							flag_metadada = 1;
							xSemaphoreGive(xSemBinSendImage);
							break;

						case 0x48:		// Iniciar transmissão da imagem
							FbRadio.parameter[0] = 'I';	FbRadio.parameter[1] = 'S';	FbRadio.parameter[2] = 'D';
							message_repeat[0] = 16;
							message_repeat[1] = 0x01;
							message_repeat[2] = 0x23;
							message_repeat[3] = 0x45;
							message_repeat[4] = 0x67;
							message_repeat[5] = 0x89;
							message_repeat[6] = 0xAB;
							message_repeat[7] = 0xCD;
							message_repeat[8] = 0xEF;
							xSemaphoreGive(xSemBinSendImage);
							break;

						case 0x49:		// Repetir um pacote especifico da imagem
							FbRadio.parameter[0] = 'R';	FbRadio.parameter[1] = 'P';	FbRadio.parameter[2] = 'T';
							message_repeat[0] = *(pRadioBuffer+2) | (message_repeat[0] & 0x80);
							message_repeat[1] = *(pRadioBuffer+3);
							message_repeat[2] = *(pRadioBuffer+4);
							message_repeat[3] = *(pRadioBuffer+5);
							message_repeat[4] = *(pRadioBuffer+6);
							message_repeat[5] = *(pRadioBuffer+7);
							message_repeat[6] = *(pRadioBuffer+8);
							message_repeat[7] = *(pRadioBuffer+9);
							message_repeat[8] = *(pRadioBuffer+10);
							xSemaphoreGive(xSemBinSendImage);
							break;

						case 0x4A:		// Prosseguir com o envio (todos os pacotes anteriores foram recebidos sem falha)
							FbRadio.parameter[0] = 'N';	FbRadio.parameter[1] = 'X';	FbRadio.parameter[2] = 'T';
							message_repeat[0] = 16;
							message_repeat[1] = 0x01;
							message_repeat[2] = 0x23;
							message_repeat[3] = 0x45;
							message_repeat[4] = 0x67;
							message_repeat[5] = 0x89;
							message_repeat[6] = 0xAB;
							message_repeat[7] = 0xCD;
							message_repeat[8] = 0xEF;
							xTaskNotify(xHandleImage, CONTINUE, eSetValueWithOverwrite);
							vTaskResume(xHandleImage);
							break;

						default:		// Recebeu um comando que não está configurado
							FbRadio.parameter[0] = 'S';	FbRadio.parameter[1] = 'H';	FbRadio.parameter[2] = 'T';
							break;
					}
				}
				else
				{
					state = receive;				// Se a mensagem não era para este Radio, volte para o modo de recebimento
				}
				break;
		}
//----------------------------------------------------------------------------------------------------------------------------------
	}
}

void vTaskImage( void * pvParameters )
{
	UBaseType_t uxHighWaterMarkImage;
	uxHighWaterMarkImage = uxTaskGetStackHighWaterMark( NULL );

	// Feedback
	FbImg.ID_Task[0] = 'I';		FbImg.ID_Task[1] = 'M';
	FbImg.parameter[0] = ' ';	FbImg.parameter[1] = ' ';	FbImg.parameter[2] = ' ';

	// Raspberry Pi State Machine
	enum {configRPi, finishRPi, suspendRPi, pictureRPi, sendRPi, continueRPi};
	static uint8_t msgRPi[6] = {CONFIG, FINISH, SUSPEND, PICTURE, SEND, CONTINUE};
	uint8_t feedbackUART = 0;

	// State Machine
	enum {suspend, timeout, uart, expected, unexpected, spi};
	uint8_t state = suspend;
	uint8_t ImgAction = 0;
	uint32_t CRC32 = 0, CRC32_Calc = 0;

	// HAL status
	#define _OK			0
	#define _ERR		1
	#define U_RX		0
	#define U_TX		1
	#define U_ECHO		2
	#define S_IT		0
	#define S_CRC		1

	HAL_StatusTypeDef status_UART_TX = 0, status_UART_RX = 0, status_SPI_RX = 0;
	uint8_t errCount = 0;
	_Bool flag_UART_Error[3] = {_OK,_OK,_OK} , flag_SPI_Error[2] = {_OK,_OK};

	// Semaphore Create
	xSemBinUART = xSemaphoreCreateBinary();
	xSemBinSPI = xSemaphoreCreateBinary();
	xSemBinSendImage = xSemaphoreCreateBinary();

	// Raspberry Image Buffer
	pBufferSPI = pvPortMalloc(4060);

	// Memory check
	if( (xSemBinUART == NULL) || (xSemBinSPI == NULL) || (pBufferSPI == NULL) || (xSemBinSendImage == NULL) )
		configASSERT(0);

//----------------------------------------------------------------------------------------------------------------------------------
	for(;;)
	{
		switch (state) {
			case suspend:
				if(xTaskNotifyWait(0x00, 0xFFFFFFFF, (uint32_t *) &ImgAction, 0) == pdTRUE)
				{
					state = uart;
				}
				else
				{
					uxHighWaterMarkImage = uxTaskGetStackHighWaterMark( NULL );
					vTaskSuspend(xHandleImage);
				}
				break;
//----------------------------------------------------------------------------------------------------------------------------------
			case timeout:
				state = suspend;

				if(flag_UART_Error[U_RX])														// Não houve eco, o codigo (ou o RPi) pode estar travado ou fechado
				{
					flag_UART_Error[U_RX] = _OK;												// Clear flag
					// cancela recebimento
					// envia feedback
					// tenta novamente ??
				}

				if(flag_SPI_Error[S_IT])														// SPI não recebeu nada
				{
					flag_SPI_Error[S_IT] = _OK;													// Clear flag
					// cancela recebimento
					// envia feedback
					// tenta novamente ??
				}

				break;
//----------------------------------------------------------------------------------------------------------------------------------
			case uart:																			// UART TX (2.1)
				status_UART_RX = HAL_UART_Receive_IT(&huart1, &feedbackUART, 1);				// Prepara UART para receber feedback (eco)

				switch(ImgAction) {
					case SUSPEND:																// Coloca em um estado conhecido (equivale a um reset)
						status_UART_TX = HAL_UART_Transmit_IT(&huart1, &msgRPi[suspendRPi], 1);
						break;

					case PICTURE:																// Tira foto
						status_UART_TX = HAL_UART_Transmit_IT(&huart1, &msgRPi[pictureRPi], 1);
						break;

					case SEND:																	// Recebe parte do arquivo gerado
						status_SPI_RX = HAL_SPI_Receive_IT(&hspi3, pBufferSPI, 4060);			// Prepara SPI para receber
						status_UART_TX = HAL_UART_Transmit_IT(&huart1, &msgRPi[sendRPi], 1);	// Solicita a transferencia
						break;

					case CONTINUE:
						status_SPI_RX = HAL_SPI_Receive_IT(&hspi3, pBufferSPI, 4060);				// Prepara SPI para receber
						status_UART_TX = HAL_UART_Transmit_IT(&huart1, &msgRPi[continueRPi], 1);	// Solicita a transferencia
						break;
				}

				if(status_UART_TX != HAL_OK) {				// HAL Feedback
					flag_UART_Error[U_TX] = _ERR;			// UART TX ERROR
					state = unexpected;		 }

				if(status_UART_RX != HAL_OK) {
					flag_UART_Error[U_RX] = _ERR;			// UART RX ERROR
					state = unexpected;		 }

				if(status_SPI_RX != HAL_OK)	{
					flag_SPI_Error[S_IT] = _ERR;			// SPI RX ERROR
					state = unexpected; 	}

				// UART RX (2.2)
				if(xSemaphoreTake(xSemBinUART, 15) == pdTRUE)
				{
					if(ImgAction == feedbackUART) {												// Se ouvir o eco do RPi
						state = expected;		  }
					else {
						flag_UART_Error[U_ECHO] = _ERR;											// Se chegar um eco diferente do comando solicitado
						state = unexpected;		  }
				}
				else {
					flag_UART_Error[U_RX] = _ERR;												// Se o eco não chegar
					state = timeout;			  }
				break;
//----------------------------------------------------------------------------------------------------------------------------------
			case expected:
				switch(feedbackUART) {
					case SUSPEND:
						ImgAction = PICTURE;													// Alterna comando para tirar foto
						state = uart;
						break;

					case PICTURE:
						ImgAction = SEND;														// Alterna comando para enviar o ponteiro do buffer para Task Radio
						state = uart;
						break;

					case CONTINUE:
					case SEND:
						state = spi;
						break;
				}
				break;
//----------------------------------------------------------------------------------------------------------------------------------
			case unexpected:
				state = suspend;															// Por padrão vai para o modo suspenso

				if(flag_UART_Error[U_TX])
				{
					flag_UART_Error[U_TX] = _OK;											// Clear flag
					FbImg.parameter[0] = 'A';												// Prepara feedback
				}
				if(flag_UART_Error[U_RX])
				{
					flag_UART_Error[U_RX] = _OK;											// Clear flag
					FbImg.parameter[0] = 'B';												// Prepara feedback
				}
				if(flag_SPI_Error[S_IT])
				{
					flag_SPI_Error[S_IT] = _OK;												// Clear flag
					FbImg.parameter[0] = 'C';												// Prepara feedback
				}
				if(flag_UART_Error[U_ECHO])
				{
					flag_UART_Error[U_ECHO] = _OK;											// Clear flag
					FbImg.parameter[0] = 'D';												// Prepara feedback
				}
				if(flag_SPI_Error[S_CRC])
				{
					flag_SPI_Error[S_CRC] = _OK;											// Clear flag
					ImgAction = SEND;														// Prepara para nova tentativa de recebimento
					state = uart;

					if(errCount++ > 3)														// Tenta novamente mais 2x
					{
						errCount = 0;														// Zera contagemd e erros
						FbImg.parameter[0] = 'E';											// Prepara feedback, não foi possivel obter uma mensagem sem erros
						state = suspend;													// Suspende operação
					}
				}
				break;
//----------------------------------------------------------------------------------------------------------------------------------
			case spi:
				if(xSemaphoreTake(xSemBinSPI, 1000) == pdTRUE)
				{
					CRC32 = *(pBufferSPI + 4056) << 24 | *(pBufferSPI + 4057) << 16 | *(pBufferSPI + 4058) << 8 | *(pBufferSPI + 4059);
					CRC32_Calc = HAL_CRC_Calculate(&hcrc, (uint32_t *) pBufferSPI, 4056) ^ 0xFFFFFFFF;

					if(CRC32 == CRC32_Calc)
					{	// CRC OK
						FbImg.parameter[0] = 'C';	FbImg.parameter[1] = 'R';	FbImg.parameter[2] = 'C';
						xSemaphoreGive(xSemBinSendImage);
						xTaskNotify(xHandleRadio, 0x0B, eSetValueWithOverwrite);
						state = suspend;
					}
					else
					{	// CRC ERROR
						flag_SPI_Error[S_CRC] = _ERR;					// CRC falhou, a mensagem transferida via SPI contem erros
						state = unexpected;
					}
				}
				else
				{
					flag_SPI_Error[S_IT] = _ERR;						// A interrupção não ocorreu dentro do intervalo de tempo
					state = timeout;
				}
				break;
		}
//----------------------------------------------------------------------------------------------------------------------------------
	}
}

void vTaskControl( void * pvParameters )
{
	FbCtrl.ID_Task[0] = 'C';	FbCtrl.ID_Task[1] = 'T';
	FbCtrl.parameter[0] = ' ';	FbCtrl.parameter[1] = ' ';	FbCtrl.parameter[2] = ' ';

	vTaskSuspend(xHandleControl);

	for(;;)
	{
		osDelay(1);
	}
}

void vTaskDeploy( void * pvParameters )
{
	/*			SEQUENCIA DE FUNCIONAMENTO
	 *
	 * 		1. Aciona o resistor de aquecimento
	 * 		2. Aguarda (timer ou switch) (libera task)
	 * 		3. Desliga Heater
	 * 		4. Coloca o resultado numa Queue
	 * 		5. Se o resultado foi positivo, apague a task, se não, suspenda e rearme
	 *
	 */

	UBaseType_t uxHighWaterMarkDeploy;
	uxHighWaterMarkDeploy = uxTaskGetStackHighWaterMark( NULL );

	FbDpl.ID_Task[0] = 'D'; 		FbDpl.ID_Task[1] = 'P';
	FbDpl.parameter[0] = 'C';	FbDpl.parameter[1] = ' ';		FbDpl.parameter[2] = ' ';

	uint16_t tempo_ligado = 1000;
	uint8_t status_deploy = FOLDED;

	// Fica suspensa até o momento do deploy ser solicitado
	vTaskSuspend(xHandleDeploy);

	for(;;)
	{
		if(status_deploy != FULLY_DEPLOYED)																// se não está aberto
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);									// Liga elemento de aquecimento
			status_deploy = DEPLOYING;	FbDpl.parameter[0] = 'W';										// atualiza status para abrindo
			xTaskNotifyWait(0x00, 0xFFFFFFFF, (uint32_t *) &status_deploy, tempo_ligado);				// Aguarda um certo tempo

			if(status_deploy == 3) {																	// Atualizado via interrupção externa
				FbDpl.parameter[0] = 'S'; }																// sucesso, aberto
			else {
				status_deploy = NOT_DEPLOYED;	FbDpl.parameter[0] = 'F'; }											// falhou, continua fechado

			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);									// Desliga elemento de aquecimento
		}
//		else
//			vTaskDelete(xHandleDeploy);				// Deleta a task

		uxHighWaterMarkDeploy = uxTaskGetStackHighWaterMark( NULL );
		vTaskSuspend(xHandleDeploy);

	}
}

void vTaskMemory( void * pvParameters )
{
	FbMmry.ID_Task[0] = 'M';	FbMmry.ID_Task[1] = 'R';
	FbMmry.parameter[0] = ' ';	FbMmry.parameter[1] = ' ';	FbMmry.parameter[2] = ' ';

//	vTaskSuspend(xHandleMemory);

	for(;;)
	{
		osDelay(1000);
	}
}

void vTaskSensor( void * pvParameters )
{
	FbSensor.ID_Task[0] = 'S';		FbSensor.ID_Task[1] = 'R';
	FbSensor.parameter[0] = ' ';	FbSensor.parameter[1] = ' ';	FbSensor.parameter[2] = ' ';

//	vTaskSuspend(xHandleSensor);

	for(;;)
	{
		osDelay(100);
	}
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StandbyTask */
/**
  * @brief  Function implementing the standbyTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StandbyTask */
void StandbyTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	vTaskSuspend(standbyTaskHandle);
  /* Infinite loop */
	for(;;)
	{
		osDelay(1000);
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
//  if (htim->Instance == TIM7) {
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//  }
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
