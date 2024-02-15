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
#include "NMEA.h"
#include "UartRingbuffer.h"
#include <stdio.h>
#include <string.h>

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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void initGPS();
int read_GPS(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

GPSSTRUCT gpsData;
int flagGGA = 0, flagRMC = 0;
char lcdBuffer1 [50];
char lcdBuffer2 [50];
char start_GPS_command[15] = "AT+GPS=1\n\r";
char received_data[100]; // hold data received from GPS
int data; // received character from uart ring buffer
char gga[10] = "GGA";
char buffer[100] = "+GPSRD:$GNGGA,090553.000,0105.8374,S,03701.2340,E,1,11,0.76,1553.9,M,-23.9,M,,*76";
int VCCTimeout = 5000; // GGA or RMC will not be received if the VCC is not sufficient

//char received_data[100];
char GGA[100]; // hold GGA data from GPS
char RMC[100];

void initGPS(){
	Uart_sendstring(start_GPS_command);
}

/**
 * @brief GPS read stuff
 * @param none
 * @retval int
 */
int read_GPS(void) {
	/*  GPS Stuff */
	  if (Wait_for("GGA") == 1)
	  {
		  VCCTimeout = 5000;  // Reset the VCC Timeout indicating the GGA is being received
		  Copy_upto("*", GGA);
		  if (decodeGGA(GGA, &gpsData.ggastruct) == 0) flagGGA = 2;  // 2 indicates the data is valid
		  else flagGGA = 1;  // 1 indicates the data is invalid
	  }

	  if (Wait_for("RMC") == 1)
	  {
		  VCCTimeout = 5000;  // Reset the VCC Timeout indicating the RMC is being received
		  Copy_upto("*", RMC);
		  if (decodeRMC(RMC, &gpsData.rmcstruct) == 0) flagRMC = 2;  // 2 indicates the data is valid
		  else flagRMC = 1;  // 1 indicates the data is invalid
	  }

	  if ((flagGGA == 2) | (flagRMC == 2))
	  {
		  memset(timeBuffer, '\0', 50);
		  sprintf (timeBuffer, "%02d:%02d:%02d, %02d%02d%02d", gpsData.ggastruct.tim.hour, \
				  gpsData.ggastruct.tim.min, gpsData.ggastruct.tim.sec, gpsData.rmcstruct.date.Day, \
				  gpsData.rmcstruct.date.Mon, gpsData.rmcstruct.date.Yr);
		  memset(posBuffer, '\0', 50);
		  sprintf (posBuffer, "%.2f%c, %.2f%c  ", gpsData.ggastruct.lcation.latitude, gpsData.ggastruct.lcation.NS,\
				  gpsData.ggastruct.lcation.longitude, gpsData.ggastruct.lcation.EW);
		  return 1;
	  }
	  else if ((flagGGA == 1) | (flagRMC == 1))
	  {
		  Uart_sendstring("no fix");
		  return 0;
	  }

	  if (VCCTimeout <= 0)
	  {
		  VCCTimeout = 5000;  // Reset the timeout

		  //reset flags
		  flagGGA =flagRMC =0;

		  Uart_sendstring("Low VCC");
		  return 0;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Ringbuf_init();
  //initGPS();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  // simulate data coming from the GPS here
	  // GGA sentence
	  //HAL_UART_Receive(&huart2, (uint8_t*)"+GPSRD:$GNGGA,090553.000,0105.8374,S,03701.2340,E,1,11,0.76,1553.9,M,-23.9,M,,*76", strlen("+GPSRD:$GNGGA,090553.000,0105.8374,S,03701.2340,E,1,11,0.76,1553.9,M,-23.9,M,,*76"), 1000);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)"Waiting\n\r", 15, 100);

	  if(Wait_for("GGA") == 1) {
		  VCCTimeout = 5000;
		  Copy_upto("*", GGA);
		  if(decodeGGA(GGA, &gpsData.ggastruct) == 0) flagGGA = 2; // 2 indicates the data is valid

		  else flagGGA = 1;
	  }

	  if (Wait_for("RMC") == 1)
	  {
		  VCCTimeout = 5000;  // Reset the VCC Timeout indicating the RMC is being received

		  Copy_upto("*", RMC);
		  if (decodeRMC(RMC, &gpsData.rmcstruct) == 0) flagRMC = 2;  // 2 indicates the data is valid
		  else flagRMC = 1;  // 1 indicates the data is invalid
	  }

	  if ((flagGGA == 2) | (flagRMC == 2)) {

		  sprintf (lcdBuffer1,
				  "%02d:%02d:%02d, %02d%02d%02d\r\n",
				  gpsData.ggastruct.tim.hour,
				  gpsData.ggastruct.tim.min,
				  gpsData.ggastruct.tim.sec,
				  gpsData.rmcstruct.date.Day,
				  gpsData.rmcstruct.date.Mon,
				  gpsData.rmcstruct.date.Yr);

//		  memset(lcdBuffer, '\0', 50);

		  sprintf (lcdBuffer2,
				  "%.2f%c, %.2f%c  \r\n",
				  gpsData.ggastruct.lcation.latitude,
				  gpsData.ggastruct.lcation.NS,
				  gpsData.ggastruct.lcation.longitude,
				  gpsData.ggastruct.lcation.EW);

		  HAL_UART_Transmit(&huart1, (uint8_t*)lcdBuffer1, 50, 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)lcdBuffer2, 50, 100);

	  } else if((flagGGA == 1) | (flagRMC == 1)) {
		  HAL_UART_Transmit(&huart1, (uint8_t*)"Failed decoding. Fix not found. Please wait. \r\n", 100, 100);
	  }

	  if(VCCTimeout <= 0 ) {
		  VCCTimeout = 5000; // reset timeout

		  // reset flags
		  flagGGA = flagRMC = 0;

		  HAL_UART_Transmit(&huart1, (uint8_t*)"Failed decoding.Please check connection. \r\n", 100, 100);
	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

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
