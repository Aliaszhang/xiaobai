/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define PRINT_ERROR(fmt, args...)  LOG_ERROR("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_WARN(fmt, args...)   LOG_WARN("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_INFO(fmt, args...)   LOG_INFO("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_DEBUG(fmt, args...)  LOG_DEBUG("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_BIN(fmt, args...)    LOG_BINARY("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
#define  ARRAYSIZE         5000

uint32_t int_device_serial[3];
uint32_t int_device_memery_info;

uint8_t spi_send_array[ARRAYSIZE] = {0};
uint8_t spi_send_array_bak[ARRAYSIZE+20] = {0};
uint8_t spi_read_reg_array[12] = {0}; 

int gpio_int_flag = 0;

int msg_len = 0;
uint8_t print_msg[256] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void get_chip_serial_num(void);
void get_chip_memery_info(void);
void adxl355_init(uint8_t range, uint8_t odr, uint8_t fifo_len);
float adxl355_conversion_acc_data(uint8_t *data);
float adxl355_conversion_temperature(uint8_t *data);
uint32_t adler32(uint8_t *buf, uint32_t len);
uint8_t spi_read_byte(uint8_t addr);
int spi_write_byte(uint8_t addr, uint8_t data);
int spi_write_multipe_bytes(uint8_t start_addr, uint8_t *txdata, uint8_t len);
int spi_read_multipe_bytes(uint8_t start_addr, uint8_t *rxdata, uint8_t len);
char *my_basename(char *s);
int log_print(void);
void HAL_RTC_GetTime_and_Date(RTC_DateTypeDef *s_date, RTC_TimeTypeDef *s_time);
void HAL_RTC_SetTime_and_Date(RTC_DateTypeDef *s_data, RTC_TimeTypeDef *s_time);
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
  int i;
	HAL_StatusTypeDef ret;
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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
	MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  PRINT_INFO("start stm32f103vct6\r\n");
	
	/* get chip serial number */
	get_chip_serial_num();

	/* printf CPU unique device id */
  PRINT_INFO("The CPU Unique Device ID:[%X-%X-%X]\r\n", int_device_serial[2], int_device_serial[1], int_device_serial[0]);

	/* get chip memery info */
	get_chip_memery_info();

	/* printf sdram and flash size */
	PRINT_INFO("The Flash size:%uKBytes, SRAM size:%uKBytes\r\n", (int_device_memery_info & 0xFFFF), (int_device_memery_info >> 16 & 0xFFFF));

	HAL_Delay(100);
	
  adxl355_init(XL355_RANGE_2G, XL355_ODR_500HZ, XL355_FIFO_SAMPLE_63);
	log_print();
	int count = 0;
	int pack_count = 0;
	uint32_t adler32_check_sum = 0;
		
	while(1)
	{
		if (gpio_int_flag > 0)
    {
      memset(spi_read_reg_array, 0x0, 12);
      ret = spi_read_multipe_bytes(XL355_TEMP2, &spi_read_reg_array[0], 11);

      PRINT_DEBUG("adxl355: temp:%0.2f\t\taccx:%0.2f\t\taccy:%0.2f\t\taccz:%0.2f\r\n", \
								adxl355_conversion_temperature(&spi_read_reg_array[0]), adxl355_conversion_acc_data(&spi_read_reg_array[2]), \
								adxl355_conversion_acc_data(&spi_read_reg_array[5]), adxl355_conversion_acc_data(&spi_read_reg_array[8]));
     
      ret = spi_read_multipe_bytes(XL355_FIFO_DATA, &spi_send_array[count], 63);

      PRINT_DEBUG("count:%04d, gpio_int_flag: %d\r\n", count, gpio_int_flag);		

      for (i = (count+2); i < (count+63); i+=9)
      {
          if ((spi_send_array[i] & 0x01) == 0x01)
          {
              PRINT_DEBUG("adxl355: %d\t\taccx%0.2f\t\taccy:%0.2f\t\taccz:%0.2f\r\n", i, \
							    adxl355_conversion_acc_data(&spi_send_array[i-2]), adxl355_conversion_acc_data(&spi_send_array[i+1]), adxl355_conversion_acc_data(&spi_send_array[i+4]));
          }
      } 

      log_print();
			
			count += 63;
			if (count >= 4977)
			{
				count = 0 ;
				pack_count = 0;
				memset(spi_send_array_bak, 0x0, ARRAYSIZE+20);
				spi_send_array_bak[0] = 0xEF;
				spi_send_array_bak[1] = 0xEF;
				spi_send_array_bak[2] = ((4977 & 0xFF000000) >> 24);
				spi_send_array_bak[3] = ((4977 & 0x00FF0000) >> 16);
				spi_send_array_bak[4] = ((4977 & 0x0000FF00) >> 8);
				spi_send_array_bak[5] = ((4977 & 0x000000FF) >> 0);
				
				pack_count += 6;
				
				memcpy(&spi_send_array_bak[6], spi_send_array, 4977);
				pack_count += 4977;
				
				adler32_check_sum = adler32(spi_send_array_bak, pack_count);
				spi_send_array_bak[pack_count+0] = ((adler32_check_sum & 0xFF000000) >> 24);
				spi_send_array_bak[pack_count+1] = ((adler32_check_sum & 0x00FF0000) >> 16);
				spi_send_array_bak[pack_count+2] = ((adler32_check_sum & 0x0000FF00) >> 8);
				spi_send_array_bak[pack_count+3] = ((adler32_check_sum & 0x000000FF) >> 0);
				pack_count += 4;
				
				HAL_UART_Transmit_DMA(&huart3, spi_send_array_bak, pack_count);
				memset(spi_send_array, 0x0, ARRAYSIZE);
			}
			gpio_int_flag = 0;
    }

	}

  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 2000000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 2000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_INT_GPIO_Port, SPI3_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_SYNC_Pin|LED_RUN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WATCHDOG_Pin|SC200R_RST_Pin|XL355_SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SC200R_EN_GPIO_Port, SC200R_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PULSE_EN_Pin|XL355_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SC200R_PW_EN_GPIO_Port, SC200R_PW_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI3_INT_Pin */
  GPIO_InitStruct.Pin = SPI3_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI3_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE6 PE7 PE8
                           PE9 PE10 PE11 PE12
                           PE13 PE14 PE15 PE0
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_SYNC_Pin */
  GPIO_InitStruct.Pin = LED_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RUN_Pin */
  GPIO_InitStruct.Pin = LED_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC3
                           PC8 PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : WATCHDOG_Pin SC200R_EN_Pin SC200R_RST_Pin */
  GPIO_InitStruct.Pin = WATCHDOG_Pin|SC200R_EN_Pin|SC200R_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : XL355_SPI_CS_Pin */
  GPIO_InitStruct.Pin = XL355_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(XL355_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 PB15 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_EN_Pin GPS_RST_Pin */
  GPIO_InitStruct.Pin = GPS_EN_Pin|GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD4 PD5
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : XL355_INT1_Pin */
  GPIO_InitStruct.Pin = XL355_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(XL355_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XL355_SYNC_Pin */
  GPIO_InitStruct.Pin = XL355_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(XL355_SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PULSE_EN_Pin */
  GPIO_InitStruct.Pin = PULSE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PULSE_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XL355_EN_Pin */
  GPIO_InitStruct.Pin = XL355_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XL355_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SC200R_PW_EN_Pin */
  GPIO_InitStruct.Pin = SC200R_PW_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SC200R_PW_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*!
    \brief      get chip serial number
    \param[in]  none
    \param[out] none
    \retval     none
*/
void get_chip_serial_num(void)
{
    int_device_serial[0] = *(__IO uint32_t*)(0x1FFFF7E8);
    int_device_serial[1] = *(__IO uint32_t*)(0x1FFFF7EC);
    int_device_serial[2] = *(__IO uint32_t*)(0x1FFFF7F0);
}

/*!
    \brief      get chip sdram and flash info
    \param[in]  none
    \param[out] none
    \retval     none
*/
void get_chip_memery_info(void)
{
    int_device_memery_info = *(__IO uint32_t*)(0x1FFFF7E0);
}

#define BASE 65521
uint32_t adler32(uint8_t *buf, uint32_t len)
{

   uint32_t adler = 1;
   uint32_t s1    = (adler >> 0) & 0xFFFF;
   uint32_t s2    = (adler >> 16) & 0xFFFF;
   uint32_t i;

   for (i = 0; i < len; i++)
   {
      s1 = (s1 + buf[i]) % BASE;
      s2 = (s2 + s1) % BASE;
//      printf("s2:%#x, s1:%#x\n", s2, s1);
   }

   return (s2 << 16) + s1;
}

/* ----------------------xl355 function------------------------------------*/
int spi_write_byte(uint8_t addr, uint8_t data)
{
    uint8_t write_address = addr << 1;

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &write_address, 1, 0xFF) == HAL_OK)
    {
        HAL_SPI_Transmit(&hspi1, &data, 1, 0xFF);
    }

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    return 0;
}

uint8_t spi_read_byte(uint8_t addr)             //read 1 Byte data
{
    uint8_t readbuff = 0xFF;
    uint8_t read_address = addr << 1;
    read_address |= 0x01;

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &read_address, 1, 0xFF) == HAL_OK)
    {
        HAL_SPI_Receive(&hspi1, &readbuff, 1, 0xFFF);
    }

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);

    return readbuff;
}

int spi_read_multipe_bytes(uint8_t start_addr, uint8_t *rxdata, uint8_t len)
{
    uint8_t read_address = ((start_addr << 1) | 0x01);

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &read_address, 1, 0xFF) != HAL_OK)
    {
        memset(print_msg, 0x0, 256);
        msg_len = snprintf((char *)print_msg, 256, "\r\nadxl355 WRITE register %u failed\r\n", start_addr);
        HAL_UART_Transmit(&huart1, print_msg, msg_len, 1000);
        HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
        return -1;
    }
    if (HAL_SPI_Receive(&hspi1, rxdata, len, 0xFFF) != HAL_OK)
    {
        memset(print_msg, 0x0, 256);
        msg_len = snprintf((char *)print_msg, 256, "\r\nadxl355 read %u bytes from register %u failed\r\n", len, start_addr);
        HAL_UART_Transmit(&huart1, print_msg, msg_len, 1000);
        HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
        return -1;
    }
    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
    return 0;
}

int spi_write_multipe_bytes(uint8_t start_addr, uint8_t *txdata, uint8_t len)
{
    uint8_t write_address = (start_addr << 1);

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &write_address, 1, 0xFF) != HAL_OK)
    {
        memset(print_msg, 0x0, 256);
        msg_len = snprintf((char *)print_msg, 256, "\r\nadxl355 WRITE register %u failed\r\n", start_addr);
        HAL_UART_Transmit(&huart1, print_msg, msg_len, 1000);
        HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
        return -1;
    }
    if (HAL_SPI_Transmit(&hspi1, txdata, len, 0xFFF) != HAL_OK)
    {
        memset(print_msg, 0x0, 256);
        msg_len = snprintf((char *)print_msg, 256, "\r\nadxl355 write %u bytes to register %u failed\r\n", len, start_addr);
        HAL_UART_Transmit(&huart1, print_msg, msg_len, 1000);
        HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
        return -1;
    }
    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
    return 0;
}

void adxl355_init(uint8_t range, uint8_t odr, uint8_t fifo_len)
{    
    
    spi_write_byte(XL355_RESET, 0x52);
    spi_write_byte(XL355_RANGE, range); // 0xC1
	  spi_write_byte(XL355_FIFO_SAMPLES, fifo_len); // 0x3F: 63: 7x9
    spi_write_byte(XL355_INT_MAP, 0x02); // FULL_EN1
    spi_write_byte(XL355_FILTER, odr); // 2000hz
    spi_write_byte(XL355_SYNC, 0x00);
//    spi_write_byte(XL355_SELF_TEST, 0x03);

	  uint8_t tmp;
    tmp = spi_read_byte(XL355_RANGE);
    if (tmp != range)
    {
        PRINT_WARN("xl355 range set fail(%x:%x)\r\n", range, tmp);
    }
    tmp = spi_read_byte(XL355_SYNC);
    if (tmp != 0x00)
    {
        PRINT_WARN("xl355 sync set fail(%x:%x)\r\n", 0x00, tmp);
    }
    tmp = spi_read_byte(XL355_FILTER);
    if (tmp != odr)
    {
        PRINT_WARN("xl355 odr set fail(%x:%x)\r\n", odr, tmp);
    }
    tmp = spi_read_byte(XL355_FIFO_SAMPLES);
    if (tmp != fifo_len)
    {
        PRINT_WARN("xl355 fifo samples set fail(%x:%x)\r\n", fifo_len, tmp);
    }
    tmp = spi_read_byte(XL355_INT_MAP);
    if (tmp != 0x02)
    {
        PRINT_WARN("xl355 int_map set fail(%x:%x)\r\n", 0x02, tmp);
    }

    if (spi_read_byte(XL355_PARTID) != 0xED)
    {
        PRINT_ERROR("adxl355 read register failed\r\n");
    }
    else
    {
        uint8_t DEVID_AD, DEVID_MST, REVID;
        DEVID_AD = spi_read_byte(XL355_DEVID_AD);
        DEVID_MST = spi_read_byte(XL355_DEVID_MST);
        REVID = spi_read_byte(XL355_REVID);
        PRINT_INFO("adxl355 ADIID:%u, MEMSID:%u, REVID:%u\r\n", DEVID_AD, DEVID_MST, REVID);

        uint8_t pwrctl = spi_read_byte(XL355_POWER_CTL);
        pwrctl &= 0xFE;

        spi_write_byte(XL355_POWER_CTL,pwrctl);
        pwrctl = spi_read_byte(XL355_POWER_CTL);
        return;
    }
}

float adxl355_conversion_acc_data(uint8_t *data)
{
  	uint32_t acc_raw;      // register data
    int32_t acc_actual;    // 
    float acc_float;       // 

    acc_raw = (((data[0] << 16) | (data[1] << 8) | data[2]) >> 4);
          
    if(acc_raw >= 0x80000)
    {
        acc_actual = -((~acc_raw & 0x3FFFF) + 1);
    }
    else
    {
        acc_actual = acc_raw;
    }
    
    acc_float = acc_actual * 0.0039; // 2g: scale factor 3.9 ug/LSB
    // acc_float = acc_actual * 0.0039; // 4g: scale factor 7.8 ug/LSB
    // acc_float = acc_actual * 0.0039; // 8g: scale factor 15.6 ug/LSB
    
    return acc_float;
}

float adxl355_conversion_temperature(uint8_t *data)
{
    uint16_t reg;
    float temp;

    reg = (((data[0] & 0x0F) << 8) | data[1]);
    temp = 25 - ((reg - 1852) / (9.05));
 
    return temp;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == XL355_INT1_Pin)
    {
        /* Toggle LED1 */
        gpio_int_flag++;

    }
}

/* ------------my log function---------------- */
#define MAX_ONE_LOG_SIZE   (250)									// length of one log message
#define MAX_LOG_BUFF_SIZE  (MAX_ONE_LOG_SIZE * 20) // total buffer length of log

char log_msg_buf[MAX_LOG_BUFF_SIZE];
char log_msg_send[MAX_LOG_BUFF_SIZE];
uint16_t log_msg_len = 0;  //needn`t mutex

char *my_basename(char *s)
{
    char *p_ret;
    p_ret = strrchr(s, '/');
    if (p_ret)
    {
        return (p_ret + 1);
    }
    return s;
}

int log_message(log_level_e level, const char *level_str, const char *fmt, ...)
{
    char msg_buf[MAX_ONE_LOG_SIZE] = {0};
    int len = 0;
    
    va_list ap;
    RTC_DateTypeDef rtc_date;
    RTC_TimeTypeDef rtc_time;

    if (level < LOG_LEVEL_DEBUG)
    {
        return 0;
    }

    if (log_msg_len+100 >= MAX_LOG_BUFF_SIZE)
    {
      return 1;
    }
    HAL_RTC_GetTime_and_Date(&rtc_date, &rtc_time);

    len = snprintf(msg_buf, MAX_ONE_LOG_SIZE, "%4u/%02u/%02u %02u:%02u:%02u.%03u[%5.5s] ",
                                    (2000 + rtc_date.Year), rtc_date.Month, rtc_date.Date, \
		rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, 1000, level_str); // todo : how to get ms
    va_start(ap, fmt);
    len += vsnprintf(&msg_buf[len], (MAX_ONE_LOG_SIZE - len), fmt, ap);
    va_end(ap);

    if (len > 0)
    {
      if ((log_msg_len + len) < MAX_LOG_BUFF_SIZE)
      {
        memcpy(&log_msg_buf[log_msg_len], msg_buf, len);
        log_msg_len += len;
        return 0;
      }
      else
      {
        log_msg_buf[log_msg_len] = 0;
        log_msg_len = MAX_LOG_BUFF_SIZE;
      }
    }

    return -1;
}

int log_binary(log_level_e level, const char *fmt, ...)
{
    va_list ap;
    char msg_buf[MAX_ONE_LOG_SIZE] = {0};
    int len = 0;

    if (level < LOG_LEVEL_DEBUG) 
    {
        return 0;
    }

    if (log_msg_len+100 >= MAX_LOG_BUFF_SIZE)
    {
      return 1;
    }

    va_start(ap, fmt);
    len = vsnprintf((char *)msg_buf, MAX_ONE_LOG_SIZE, fmt, ap);
    va_end(ap);

    if (len > 0)
    {
      if ((log_msg_len + len) < MAX_LOG_BUFF_SIZE)
      {
        memcpy(&log_msg_buf[log_msg_len], msg_buf, len);
        log_msg_len += len;
        return 0;
      }
      else
      {
        log_msg_buf[log_msg_len] = 0;
        log_msg_len = MAX_LOG_BUFF_SIZE;
      }
    }

    return -1;
}


int log_print(void)
{
    if (log_msg_len > 0) 
    {
        memcpy(log_msg_send, log_msg_buf, MAX_LOG_BUFF_SIZE);
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)log_msg_send, strlen(log_msg_send));
        memset(log_msg_buf, 0x0, MAX_LOG_BUFF_SIZE);
        log_msg_len = 0;
        return 0;
    }
    return -1;
}

/*-----------------rtc function ------------------------*/
void HAL_RTC_GetTime_and_Date(RTC_DateTypeDef *s_date, RTC_TimeTypeDef *s_time)
{
  if (s_time != NULL)
    HAL_RTC_GetTime(&hrtc, s_time, RTC_FORMAT_BIN);

  if (s_date != NULL)
    HAL_RTC_GetDate(&hrtc, s_date, RTC_FORMAT_BIN);
}

void HAL_RTC_SetTime_and_Date(RTC_DateTypeDef *s_data, RTC_TimeTypeDef *s_time)
{
  if (s_time != NULL)
    HAL_RTC_SetTime(&hrtc, s_time, RTC_FORMAT_BIN);
  if (s_data != NULL)
    HAL_RTC_SetDate(&hrtc, s_data, RTC_FORMAT_BIN);
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
