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
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_MAX_LEN (3000u)
// 震动数据接收缓存，每个点9bytes, 收满500个点发�?�一次数�???????
#define XL355_SAMPLE_COUNT      (90 * 50)
#define PACK_LEN	(19)
// 200是为添加的包头和校验码留的空�???????
#define  ARRAYSIZE         (XL355_SAMPLE_COUNT + 200)

typedef enum {
    LOG_LEVEL_DEBUG = 1,
    LOG_LEVEL_INFO = 2,
    LOG_LEVEL_WARN = 3,
    LOG_LEVEL_ERROR = 4,
    LOG_LEVEL_NONE = 5
} log_level_e;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t spi_recv_buffer[ARRAYSIZE] = {0};
uint8_t spi_send_buffer[ARRAYSIZE] = {"SUCCESS"};
const uint16_t recv_len = (XL355_SAMPLE_COUNT + PACK_LEN);
// 发�?�震动数据的buffer
uint8_t sensor_data[ARRAYSIZE] = {0};
// 从xl355寄存器读�???????次温度传感器�??????? xyz 3轴的震动数据用的buffer
uint8_t spi_read_reg_array[12] = {0};

int xl355_fifo_full_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
float adxl355_conversion_acc_data(uint8_t *data);
//int log_message(log_level_e level, const char *level_str, const char *fmt, ...);
int log_binary(log_level_e level, const char *fmt, ...);
char *my_basename(char *s);
int log_print(void);
uint32_t adler32(uint8_t *buf, uint32_t len);
int unpackage_xl355_raw_data(uint8_t *in_buf, uint8_t *out_buff, uint16_t in_len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#define LOG_ERROR(fmt, args...) log_message(LOG_LEVEL_ERROR, "error", fmt, ##args)
//#define LOG_WARN(fmt, args...) log_message(LOG_LEVEL_WARN, "warn", fmt, ##args)
//#define LOG_INFO(fmt, args...) log_message(LOG_LEVEL_INFO, "info", fmt, ##args)
//#define LOG_DEBUG(fmt, args...) log_message(LOG_LEVEL_DEBUG, "debug", fmt, ##args)
#define LOG_BINARY(fmt, args...) log_binary(LOG_LEVEL_DEBUG, fmt, ##args)

//#define PRINT_ERROR(fmt, args...)  LOG_ERROR("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
//#define PRINT_WARN(fmt, args...)   LOG_WARN("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
//#define PRINT_INFO(fmt, args...)   LOG_INFO("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
//#define PRINT_DEBUG(fmt, args...)  LOG_DEBUG("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_BIN(fmt, args...)    LOG_BINARY("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_StatusTypeDef ret;
	int i;
	int data_len;
	float x,y,z;
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  PRINT_BIN("start app\r\n");
  log_print();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (xl355_fifo_full_flag > 0)
	  {
		  PRINT_BIN("xl355_fifo_full_flag:%d\r\n", xl355_fifo_full_flag);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  ret = HAL_SPI_Receive(&hspi2, spi_recv_buffer, recv_len, 1000);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		  if (ret != HAL_OK)
		  {
			  PRINT_BIN("SPI SEND FAIL:%d\r\n", ret);
		  }
		  else
		  {
		      HAL_UART_Transmit_DMA(&huart2, spi_recv_buffer, recv_len);
			  data_len = unpackage_xl355_raw_data(spi_recv_buffer, sensor_data, recv_len);
			  if (data_len < 0)
			  {
				  PRINT_BIN("sensor data len error\r\n");
			  }
			  else
			  {
				  i = 0;
//				  for (i = 0; i < data_len; i+=9)
				  {
//					  if ((sensor_data[i+2] & 0x01) == 0x01)
					  {
						  x = adxl355_conversion_acc_data(&sensor_data[i]);
						  y = adxl355_conversion_acc_data(&sensor_data[i+3]);
						  z = adxl355_conversion_acc_data(&sensor_data[i+6]);
						  PRINT_BIN("[%04d]accx:%0.2f\t accy:%0.2f\t accz:%0.2f\r\n", i, x, y, z);
					  }
				  }
			  }
		  }
		  xl355_fifo_full_flag = 0;
	  }
	  log_print();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 120;
  RCC_OscInitStruct.PLL.PLLR = 5;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
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
  huart1.Init.BaudRate = 2000000;
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
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_6)
	{
		xl355_fifo_full_flag++;
	}
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

int unpackage_xl355_raw_data(uint8_t *in_buf, uint8_t *out_buff, uint16_t in_len)
{
    int pack_content_len = 0;
    int raw_data_len = 0;
    uint8_t cmd = 0;
    uint32_t adler32_check_sum = 0;
    uint32_t adler32_code = 0;
    uint32_t microseconds;
    uint32_t seconds;

    if (in_buf[0] != 0xEF || in_buf[1] != 0xEF)
    {
    	PRINT_BIN("header not right %#x %#x\r\n", in_buf[0], in_buf[1]);
    	return -1;
    }

    pack_content_len = ((in_buf[2] << 24) | (in_buf[3] << 16) | (in_buf[4] << 8) | in_buf[5]);
    if (pack_content_len <= 9 || in_len < pack_content_len)
    {
    	PRINT_BIN("content len not right %#x : %#x\r\n", pack_content_len, in_len);
    	return -1;
    }
    raw_data_len = pack_content_len - 9;
    PRINT_BIN("raw data len %d\r\n", raw_data_len);

    adler32_code = ((in_buf[in_len - 4] << 24) | (in_buf[in_len - 3] << 16) | (in_buf[in_len - 2] << 8) | in_buf[in_len - 1]);
    adler32_check_sum = adler32(in_buf, in_len - 4);

    if (adler32_code != adler32_check_sum)
    {
    	PRINT_BIN("adler32 code not equal %#x:%#x\r\n", adler32_code, adler32_check_sum);
    	return -1;
    }

    cmd = in_buf[6];
    PRINT_BIN("cmd: %#x\r\n", cmd);

    seconds      = ((in_buf[7] << 24)  | (in_buf[8] << 16)  | (in_buf[9] << 8)  | in_buf[10]);
    microseconds = ((in_buf[11] << 24) | (in_buf[12] << 16) | (in_buf[13] << 8) | in_buf[14]);
    PRINT_BIN("time:%u.%u\r\n", seconds, microseconds/1000);


    memcpy(out_buff, &in_buf[15], raw_data_len);

    return raw_data_len;
}

/* ------------my log function---------------- */
#define MAX_ONE_LOG_SIZE   (250)									// length of one log message
#define MAX_LOG_BUFF_SIZE  (MAX_ONE_LOG_SIZE * 30) // total buffer length of log

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

/*int log_message(log_level_e level, const char *level_str, const char *fmt, ...)
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
		rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, ((sys_run.time_5ms_count%200) * 5), level_str); // todo : how to get ms
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
}*/

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
