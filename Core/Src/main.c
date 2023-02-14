/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "lv_impl.h"

#include <stdio.h>
#include <stdbool.h>
//#include "nrf24l01.h"
#include "nrf24.h"
#include "ili9341.h"
//#include "oled.h"
//#include "xpt2046.h"
//#include "img.h"
#include "add.h"
#include "radio_type.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	ADC_CHANNEL_RIGHT_Y,
	ADC_CHANNEL_RIGHT_X,
	ADC_CHANNEL_LEFT_X,
	ADC_CHANNEL_LEFT_Y,
	ADC_CHANNEL_BATTERY,
	ADC_CHANNELS_NUM
} adc_channels;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
#define ADC_CHANNELS_NUM                                         5
#define ADC_CHANNEL_LEFT_X                                       3
#define ADC_CHANNEL_RIGHT_X                                      0
#define ADC_CHANNEL_LEFT_Y                                       2
#define ADC_CHANNEL_RIGHT_Y                                      1
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* Definitions for main */
osThreadId_t mainHandle;
const osThreadAttr_t main_attributes = {
  .name = "main",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for joystick */
osThreadId_t joystickHandle;
const osThreadAttr_t joystick_attributes = {
  .name = "joystick",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for radio */
osThreadId_t radioHandle;
const osThreadAttr_t radio_attributes = {
  .name = "radio",
  .stack_size = 2848 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for display */
osThreadId_t displayHandle;
const osThreadAttr_t display_attributes = {
  .name = "display",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ackMessage */
osMessageQueueId_t ackMessageHandle;
const osMessageQueueAttr_t ackMessage_attributes = {
  .name = "ackMessage"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
void mainTask(void *argument);
void joystickTask(void *argument);
void radioTask(void *argument);
void displayTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t TxpipeAddrs = 0x11223344AA;
//char myTxData[32] = "Hello World!\r\n";
char AckPayload[32];
uint8_t myTxData[]="Hello World!\r\n";
uint8_t welcome[]="Hello from console STM32F411\r\n";
uint32_t global_cnt = 0;


uint16_t adcData[ADC_CHANNELS_NUM];
uint16_t adcVoltagemV[ADC_CHANNELS_NUM];
float adcVoltage[ADC_CHANNELS_NUM];
char lcd_joystick_str[32];


joystick_t joystick;

uint8_t flag_press = 0;

uint32_t counter = 0;
uint8_t test_cnt = 0;
lv_obj_t * arc;


const uint64_t pipe1 = 0xE8E8F0F0E2LL;
const uint64_t pipe5 = 0xE8E8F0F0E7LL;


uint8_t nrf_data[32] = {0};
uint8_t nrf_ack[32] = {0};


uint8_t left_motor;
uint8_t right_motor;

pump_states_t pump_command;

camera_states_t camera_command;

int16_t x_offset;
int16_t y_offset;

//SemaphoreHandle_t common_spi_mutex;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  __HAL_SPI_ENABLE(DISP_SPI_PTR);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, ADC_CHANNELS_NUM);
/*
  NRF24_begin(GPIOB, GPIO_PIN_1, GPIO_PIN_0, hspi1);
  nrf24_DebugUART_Init(huart1);

//  ssd1306_Init(&hi2c1);



  NRF24_stopListening();
  NRF24_openWritingPipe(TxpipeAddrs);
  NRF24_setAutoAck(true);
  NRF24_setChannel(52);
  NRF24_setPayloadSize(32);

  NRF24_enableDynamicPayloads();
  NRF24_enableAckPayload();


  printRadioSettings();

  HAL_UART_Transmit_IT(&huart1, &welcome, strlen(welcome));
*/

  	nrf_init(&hspi2, RF24_PA_MAX, RF24_250KBPS, 120, &huart1);


    lv_init();
    lv_disp_init();
    lv_menu();

/*
    ILI9341_Init();

    ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);

    ILI9341_Fill_Screen(MYFON);

    ILI9341_WriteString(10, 10, "Hello World", Font_7x10, WHITE, MYFON);
//    ILI9341_WriteString(20, 30, "Hello World", Font_11x18, WHITE, MYFON);
//    ILI9341_WriteString(30, 60, "Hello World", Font_16x26, BLUE, DARKGREEN);

    char txt_buf[] = "Hello Lviv";
    ILI9341_WriteString(40, 40, txt_buf, Font_16x26, YELLOW, MYFON);


    uint16_t size_img = sizeof(img_logo);
    ILI9341_Draw_Image(img_logo, 150, 100, 100, 100, size_img);
    ILI9341_WriteString(30, 120, "Odesa mama", Font_11x18, WHITE, MYFON);
*/

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

  /* Create the queue(s) */
  /* creation of ackMessage */
  ackMessageHandle = osMessageQueueNew (32, 8, &ackMessage_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */


  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of main */
  mainHandle = osThreadNew(mainTask, NULL, &main_attributes);

  /* creation of joystick */
  joystickHandle = osThreadNew(joystickTask, NULL, &joystick_attributes);

  /* creation of radio */
  radioHandle = osThreadNew(radioTask, NULL, &radio_attributes);

  /* creation of display */
  displayHandle = osThreadNew(displayTask, NULL, &display_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILDIN_GPIO_Port, LED_BUILDIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF_CE_Pin|NRF_CSN_Pin|LCD_CS_Pin|LCD_RST_Pin
                          |LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILDIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILDIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILDIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGT_BUTTON_Pin LEFT_BUTTON_Pin */
  GPIO_InitStruct.Pin = RIGT_BUTTON_Pin|LEFT_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_BUILDIN_Pin */
  GPIO_InitStruct.Pin = BUTTON_BUILDIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_BUILDIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin LCD_CS_Pin LCD_RST_Pin
                           LCD_DC_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin|LCD_CS_Pin|LCD_RST_Pin
                          |LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
/*
    adcVoltage[0] = adcData[1] * 3.3 / 4095;
    adcVoltage[1] = adcData[2] * 3.3 / 4095;
    adcVoltage[2] = adcData[3] * 3.3 / 4095;
    adcVoltage[3] = adcData[4] * 3.3 / 4095;
    adcVoltage[4] = adcData[9] * 3.3 / 4095;
    */
//    adcVoltagemV[0] = adcData[1] * 3300 / 4095;
//    adcVoltagemV[1] = adcData[2] * 3300 / 4095;
//    adcVoltagemV[2] = adcData[3] * 3300 / 4095;
//    adcVoltagemV[3] = adcData[4] * 3300 / 4095;
    adcVoltagemV[4] = adcData[4] * 3300 / 4095 * 25 / 15;

    HAL_ADC_Stop_DMA(&hadc1);
  }

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_mainTask */
/**
  * @brief  Function implementing the main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainTask */
void mainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	lv_timer_handler();
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_joystickTask */
/**
* @brief Function implementing the joystick thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_joystickTask */
void joystickTask(void *argument)
{
  /* USER CODE BEGIN joystickTask */
	uint16_t forward;
	uint16_t left_slowdown;
	uint16_t right_slowdown;
	static bool turn_flag = false;
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, ADC_CHANNELS_NUM);

	joystick.leftX = adcData[ADC_CHANNEL_LEFT_X];
	joystick.leftY = adcData[ADC_CHANNEL_LEFT_Y];
	joystick.rightX = adcData[ADC_CHANNEL_RIGHT_X];
	joystick.rightY = adcData[ADC_CHANNEL_RIGHT_Y];

	forward = map(joystick.leftY, 2100, 4095, 0, 255);

	if (joystick.leftX < 1990){
	  left_slowdown = map(joystick.leftX, 1990, 0, 255, 0);
	} else if (joystick.leftX > 2105){
	  right_slowdown = map(joystick.leftX, 2105, 4095, 0, 255);
	} else {
	  right_slowdown = 0;
	  left_slowdown = 0;
	}

	if (forward > left_slowdown / 4){
	  left_motor = forward - left_slowdown / 4;
	} else if (forward == 0) {
      right_motor = left_slowdown / 4;
	} else {
	  left_motor = forward;
	}

	if (forward > right_slowdown / 4){
	  right_motor = forward - right_slowdown / 4;
	} else if (forward == 0) {
	  left_motor = right_slowdown / 4;
	} else {
	  right_motor = forward;
	}

	nrf_data[LEFT_MOTOR] = left_motor;
	nrf_data[RIGHT_MOTOR] = right_motor;


	if (adcData[ADC_CHANNEL_RIGHT_X] > 2500){
		pump_command = PUMP_PUSH;
	} else if (adcData[ADC_CHANNEL_RIGHT_X] < 1500){
		pump_command = PUMP_PULL;
	} else {
		pump_command = PUMP_OFF;
	}

	if (HAL_GPIO_ReadPin(RIGT_BUTTON_GPIO_Port, RIGT_BUTTON_Pin) == GPIO_PIN_RESET){
		if (turn_flag == false){
			if (camera_command == CAMERA_ON){
				camera_command = CAMERA_OFF;
			} else {
				camera_command = CAMERA_ON;
			}
			turn_flag = true;
		}
	} else {
		if (turn_flag == true){
			turn_flag = false;
		}
	}

	if (HAL_GPIO_ReadPin(LEFT_BUTTON_GPIO_Port, LEFT_BUTTON_Pin) == GPIO_PIN_RESET){

	}



/*
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		 sprintf(myTxData, "Pressed %d\r\n", counter);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		sprintf(myTxData, "Hello world %d\r\n", counter);
	}
*/
	osDelay(50);
  }
  /* USER CODE END joystickTask */
}

/* USER CODE BEGIN Header_radioTask */
/**
* @brief Function implementing the radio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_radioTask */
void radioTask(void *argument)
{
  /* USER CODE BEGIN radioTask */
	char str[64];
	uint8_t a = 1;

  /* Infinite loop */
  for(;;)
  {
	  //clear array
	  for (uint8_t i = 0; i < 32; i++){
		  nrf_data[i] = 0;
	  }

	  nrf_data[TEST_BYTE] = 0x55;

	  nrf_data[LEFT_MOTOR] = left_motor;
	  nrf_data[RIGHT_MOTOR] = right_motor;
	  nrf_data[DEPHT_OFFSET] = pump_command;
	  nrf_data[CAMERA_COMMAND] = camera_command;
	  nrf_data[LEFT_BUT] = 0;

	  nrf_openWritingPipe(pipe1);

	  if(nrf_write(&nrf_data, 32/*strlen((const char*)nrf_data)*/))
	 {
		if(nrf_isAckPayloadAvailable())
		{
			nrf_read(&nrf_ack, 32);

			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			snprintf(str, 64, "Ack: %d\n", nrf_ack);

			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

			//osMessageQueuePut(ackMessageHandle, &nrf_ack, 0, 200);
		}
	  }
	  else HAL_UART_Transmit(&huart1, (uint8_t*)"Not write 1\n", strlen("Not write 1\n"), 1000);
/*
	  nrf_openWritingPipe(pipe5);

	  for (uint8_t i = 0; i<32; i++){
		 nrf_data[i] = i+a;
		 a+=4;
		 if (a > 1000) a = 1;
	  }

	  if(nrf_write(&nrf_data, strlen((const char*)nrf_data)))
	  {
		if(nrf_isAckPayloadAvailable())
		{
			nrf_read(&nrf_ack, sizeof(nrf_ack));
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			snprintf(str, 64, "Ack: %u\n", nrf_ack);
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
		}
	  }
	  else HAL_UART_Transmit(&huart1, (uint8_t*)"Not write 5\n", strlen("Not write 5\n"), 1000);
*/
/*
	if(NRF24_write(myTxData, 32))
	{
	  NRF24_read(AckPayload, 32);
	  HAL_UART_Transmit(&huart1, (uint8_t *)"Transmitted Successfully\r\n", strlen("Transmitted Successfully\r\n"),100);
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  char myDataack[80];
	  sprintf(myDataack, "AckPayload:  %s \r\n", AckPayload);
	  HAL_UART_Transmit(&huart1, (uint8_t *)myDataack, strlen(myDataack),100);

	  osMessageQueuePut(ackMessageHandle, &AckPayload, 0, 200);
	}
*/
    osDelay(1000);
  }
  /* USER CODE END radioTask */
}

/* USER CODE BEGIN Header_displayTask */
/**
* @brief Function implementing the display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayTask */
void displayTask(void *argument)
{
  /* USER CODE BEGIN displayTask */
  uint8_t ackMessage[32] = {0};
//  char test_str[32] = {0};
  char xy_label_str[64] = {0};
  char send_data_label_str[64] = {0};
  char radio_str[64] = {0};
  char battery_str[20] = {0};
  //pressure p;
  uint16_t temp_insade;
  uint16_t battery_voltage;
  uint32_t pressure_inside;
  /* Infinite loop */
  for(;;)
  {
	//osMessageQueueGet(ackMessageHandle, &ackMessage, NULL, 200);

	set_bar(lx_bar, map(joystick.leftX, 0, 4025, 0, 100));
	set_bar(ly_bar, map(joystick.leftY, 0, 4025, 0, 100));
	set_bar(rx_bar, map(left_motor, 0, 255, 0, 100));
	set_bar(ry_bar, map(right_motor, 0, 255, 0, 100));

	sprintf(battery_str, "%dmV", adcVoltagemV[ADC_CHANNEL_BATTERY]);
	lv_label_set_text(battery_label, battery_str);

	sprintf(xy_label_str, "X: %d Y: %d | X: %d Y: %d", joystick.leftX, joystick.leftY, joystick.rightX, joystick.rightY);
	lv_label_set_text(l_xy_label, xy_label_str);

	pressure_inside = (uint32_t)nrf_ack[PRESSURE_INSIDE_0] <<  0 | (uint32_t)nrf_ack[PRESSURE_INSIDE_1] <<  8 | (uint32_t)nrf_ack[PRESSURE_INSIDE_2] << 16 | (uint32_t)nrf_ack[PRESSURE_INSIDE_3] << 24;
	temp_insade = nrf_ack[TEMP_INSIDE_MSB] | nrf_ack[TEMP_INSIDE_LSB] << 8;
	battery_voltage = nrf_ack[BATTERY_VOLTAGE_MSB] | nrf_ack[BATTERY_VOLTAGE_LSB] << 8;
	sprintf(radio_str, "T: %d, P: %d, d: %d mm, %d mV", temp_insade, pressure_inside, nrf_ack[PISTON_OFFSET], battery_voltage);
	lv_label_set_text(radio_label, radio_str);

	sprintf(send_data_label_str, "LM: %d, RM: %d, PC: %d CC: %d", left_motor, right_motor, pump_command, camera_command);
	lv_label_set_text(send_data_label, send_data_label_str);

	x_offset = (int16_t)nrf_ack[X_AXIS_MSB] <<  0 | (int16_t)nrf_ack[X_AXIS_LSB] << 8;
	y_offset = (int16_t)nrf_ack[Y_AXIS_MSB] <<  0 | (int16_t)nrf_ack[Y_AXIS_LSB] << 8;
	sprintf(send_data_label_str, "X: %d, Y: %d", x_offset, y_offset);
	lv_label_set_text(xy_label, send_data_label_str);


//	ssd1306_SetCursor(0, 0);
//	sprintf(lcd_joystick_str, "%d %d       ", joystick.leftX, joystick.leftY);
//	ssd1306_WriteString(&lcd_joystick_str, Font_11x18, White);
//	ssd1306_SetCursor(0, 25);
//	sprintf(lcd_joystick_str, "%d %d       ", joystick.rightX, joystick.rightY);
//	ssd1306_WriteString(&lcd_joystick_str, Font_11x18, White);
//	ssd1306_UpdateScreen(&hi2c1);

    osDelay(250);
  }
  /* USER CODE END displayTask */
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
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
