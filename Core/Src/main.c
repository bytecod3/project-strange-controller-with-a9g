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
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define I2C_ADDRESS        0x3E  /* Real 7 bits slave address value in Datasheet is: b0011111
                                    mean in uint8_t equivalent at 0x1F and this value can be
                                    seen in the OAR1 register in bits ADD[1:7] */
#define MASTER_REQ_READ    0x12
#define MASTER_REQ_WRITE   0x34

#define pi 3.1416

/* Buffer Used for transmission */
uint8_t aTxBuffer[] = "Received message";

/* Buffer used for Reception */
uint8_t aRxBuffer[16];
uint8_t bTransferRequest = 0;
uint8_t initTransfer = 0;


#define DEBUG 1 // set to 0 to disable debugging

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void initGPS();
int read_GPS(void);
void Read_GPS(void);
int Get_TimeElapsed(int);
int Check_GeoFence(void);
void Retrieve_EEPROM (uint32_t);
int Save_toEEPROM(uint32_t, char*);
float deg2rad(float);
float rad2deg(float);
void Set_Time(int);
int Set_Location(uint32_t);
int Set_Location(uint32_t);

// helper functions
void myprintf(UART_HandleTypeDef huart, const char* fmt, ...);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char* lineStatus[] = {"stopped", "running", "fast forward", "fast forward->12"};
int linesInd = 0;
int setLoc = 0;   // I2C flag to set current location for Geo-Fencing


GPSSTRUCT gpsData;
int flagGGA = 0, flagRMC = 0;

char lcdBuffer1 [50];
char lcdBuffer2 [50];

char start_GPS_command[15] = "AT+GPS=1\n\r";
char received_data[100]; // hold data received from GPS
char gga[10] = "GGA";

char posBuffer[50];
char timeBuffer[50];

int VCCTimeout = 5000; // GGA or RMC will not be received if the VCC is not sufficient
char no_fix_message[40] = "Decode error: No satellite fix. Please move to an open area";
char vcc_timeout_message[40] = "Decode error: Low VCC. GPS power supply should be 5V";


/** EEPROM STUFF **/
// EEPROM Addresses
uint32_t locAddress = 0x08004810;
uint32_t timeAddress = 0x08005C10;
uint32_t refAddress = 0x08004410;
char *data = "hello FLASH from ControllerTech\
			  This is a test to see how many";

char *timeG = "12:12:14";
char *dateG = "010223";
char *lat = "12.34S";
char *lon = "12.34W";
int GPSLocked; //tells if we have GPS signal locked or not

uint32_t data2[] = {0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9};

uint32_t Rx_Data[30];

char string[100];

float RxVal;

// Save seconds and minutes on timer
int secS = 0;     // Timer2 second lapsed (tick)
int minP = 0;     // Timer minute
int minD = 0;     // Minutes lapsed since failed initialization


//char received_data[100];
char GGA[100]; // hold GGA data from GPS
char RMC[100];

void initGPS(){
	Uart_sendstring(start_GPS_command);
}


void myprintf(UART_HandleTypeDef huart, const char* fmt,  ...){
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart, (uint8_t*)buffer, len, -1 );

}



/** EEPROM STUFF **/
// EEPROM Addresses
uint32_t locAddress = 0x08004810;
uint32_t timeAddress = 0x08005C10;
uint32_t refAddress = 0x08004410;
char *data = "hello FLASH from ControllerTech this is a test to see how many";

char *timeG = "12:12:14";
char *dateG = "010223";
char *lat = "12.34S";
char *lon = "12.34W";
int GPSLocked; //tells if we have GPS signal locked or not

uint32_t data2[] = {0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9};

uint32_t Rx_Data[30];

char string[100];

float RxVal;

// Save seconds and minutes on timer
int secS = 0;     // Timer2 second lapsed (tick)
int minP = 0;     // Timer minute
int minD = 0;     // Minutes lapsed since failed initialization


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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  Ringbuf_init();
  //initGPS();


  // Get current time (microseconds

  Ringbuf_init();

  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
	  /* Transfer error in reception process****/
	  Error_Handler();
  }


  HAL_Delay(1000);

  int count = 0;
  HAL_UART_Transmit(&huart1, "Trying GPS\n", 10, 100);

  while (!read_GPS() && count < 5) {
	  // wait until we have a GPS Lock
	  HAL_UART_Transmit(&huart1, "Trying GPS\n", 10, 100);
	  count++;
	  HAL_Delay(2000);
  }

  count = 0;
  while (!Check_GeoFence() && count < 3) {
	  HAL_UART_Transmit(&huart1, "Trying GeoFence\n", 15, 100);
	  // notify stolen
	  count++;
	  if (!read_GPS()) {
		  return 0;
	  }
  }

  if (count < 3) {
	  linesInd = 2;
	  Set_Time(minP);
  }
  count = 0;   // Reset count value to be re-used elsewhere

  // Start timer
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
  HAL_UART_Transmit(&huart1, "Shall we?", 9, 100);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if DEBUG == 1
	  read_GPS();

	  myprintf(huart1, posBuffer);
	  myprintf(huart1, timeBuffer);

#else
	  if (Xfer_Complete == 1) {
	 		  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
	 		  	  /* Transfer error in reception process***/
	 		  	  Error_Handler();
	 		    }
	 		  Xfer_Complete = 0;
	 	  }

	 	  // if one minute has passed, send tick
	 	  if (minP) {
	 		  minP = 0;
	 		  if (linesInd == 1) {
	 			  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
	 			  HAL_Delay(50);
	 			  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
	 			  if (!Save_toEEPROM(timeAddress, timeBuffer)){
	 				  // cannot save time to EEPROM
	 			  }
	 			  if (dayP && count < 5) {
	 				  if (read_GPS()) {
	 					  if (!Check_GeoFence()) {
	 						  // Not where it should be, do sumn
	 						  count++;
	 						  if (count == 5) stolenProtocol();
	 					  } else dayP = 0;
	 				  }
	 				  else {
	 					  // Cannot receive GPS reading
	 					  count++;
	 				  }
	 			  }
	 		  }
	 		  else if (linesInd == 0) {
	 			  if (setLoc == 1) {
	 				  if (Set_Location(locAddress)) setLoc = 0;
	 			  }
	 			  minD++;
	 			  if (read_GPS()) Set_Time(minD);
	 		  }
	 	  }

	#endif



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 7200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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


/**
 * @brief prompt from controller to save current location
 * @param locAddress GPS location from module to be saved
 * @retval ack
 */
int Set_Location(uint32_t address) {
	if (read_GPS()) {
		int i = Save_toEEPROM(address, posBuffer);
		linesInd = 1;
		return i;
	}
	else return 0;
}

/**
 * @brief Get time elapsed
 * @param delays = any added delays
 * @retval ticks = number of minute ticks to do time correction
 */
int Get_TimeElapsed(int delays) {
	/** Current time in uS */
    int hr, min, sec;
    int hrC, minC, secC;
    Retrieve_EEPROM(timeAddress);
    scanf(string, "%02d:%02d:%02d", &hr, &min, &sec);
    if (hr < 1 && min < 1 && sec < 1) return 0;
    scanf(timeBuffer, "%02d:%02d:%02d", &hrC, &minC, &secC);
    if (hrC < 1 && minC < 1 && secC < 1) return 0;
	hr = (hr >= 12) ? hr - 12 : hr;
	hr = (hrC >= 12) ? hrC - 12 : hrC;
	int total_minutes = hr * 60 + min;
	int total_minutesC = hrC * 60 + min;
	int diff = total_minutes + delays - total_minutesC;
	if (diff < 0) diff += 720;

	// time per fast-forward ticks is 1/2 sec
	// so...to correct time we will have to have to add that time
    diff += (diff / 30); // minutes convert to seconds then div by 2 since 2 ticks per sec
	return diff;
}

/**
 * @brief Sets clocks hands
 * @param none
 * @retval none
 */
void Set_Time(int tickE) {
  int ticks = Get_TimeElapsed(tickE);

  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
  if (ticks > 0) {
	  linesInd = 2;
	  for (int i = 0; i <= ticks; i++) {
			//Uart_sendstring("Onn");
			HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
			HAL_Delay (50);
			HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
			HAL_Delay (200);
	  }
	  linesInd = 1;
  }
}

/**
 * @brief Function to save latest time in EEPROM
 * @param time: time as a string
 * @retval status = successful (1) else 0
 */
int Save_toEEPROM(uint32_t address, char* time) {
	char str[100];
	int numofwords = (strlen(time)/4)+((strlen(time)%4)!=0);
	Flash_Write_Data(address , (uint32_t *)time, numofwords);
	Flash_Read_Data(address , Rx_Data, numofwords);
	Convert_To_Str(Rx_Data, str);
	if (!strcmp(time, str)) {
		return 0;
	} else return 1;
}

/**
 * @brief Retrieve from EEPROM
 * @param none
 * @retval none
 */
void Retrieve_EEPROM (uint32_t address) {
	memset(string,'\0', 50);
	int numofwords = (strlen(data)/4)+((strlen(data)%4)!=0); // Update Data value, use standard
	Flash_Read_Data(address , Rx_Data, numofwords);
	Convert_To_Str(Rx_Data, string);
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

		  sprintf (timeBuffer,
				  "%02d:%02d:%02d, %02d%02d%02d",
				  gpsData.ggastruct.tim.hour,
				  gpsData.ggastruct.tim.min,
				  gpsData.ggastruct.tim.sec,
				  gpsData.rmcstruct.date.Day,
				  gpsData.rmcstruct.date.Mon,
				  gpsData.rmcstruct.date.Yr);

		  memset(posBuffer, '\0', 50);

		  sprintf (posBuffer,
				  "%.2f%c, %.2f%c  ",
				  gpsData.ggastruct.lcation.latitude,
				  gpsData.ggastruct.lcation.NS,
				  gpsData.ggastruct.lcation.longitude,
				  gpsData.ggastruct.lcation.EW);

		  return 1;
	  }
	  else if ((flagGGA == 1) | (flagRMC == 1))
	  {
		  myprintf(huart1, no_fix_message);
		  return 0;
	  }

	  if (VCCTimeout <= 0)
	  {
		  VCCTimeout = 5000;  // Reset the timeout

		  //reset flags
		  flagGGA =flagRMC =0;

		  myprintf(huart1, vcc_timeout_message);
		  return 0;
	  }
}


/**
 * @brief function to check Geofence
 * @param none
 * @retval int
 **/
int Check_GeoFence(void) {
	float lon, lat, lonS, latS;
	char ns, ew, nsS, ewS;
	Retrieve_EEPROM(locAddress);
	scanf(string, "%.2f%c, %.2f%c  ", &lat,&ns,&lon,&ew);
	if (lat < 1 && lon < 1) return 0;
	scanf(posBuffer, "%.2f%c, %.2f%c  ", &latS,&nsS,&lonS,&ewS);
	float theta, dist;
	theta = lonS - lon;
	dist = sin(deg2rad(latS)) * sin(deg2rad(lat)) + cos(deg2rad(latS)) * cos(deg2rad(lat)) * cos(deg2rad(theta));
	dist = acos(dist);
	dist = rad2deg(dist);
	dist = dist * 60 * 1.1515;
	dist = dist * 1609.344;   // Distance in meters
	if (dist < 50) {
		return 1;
	} else return -1;
}


/**
 * @brief converts decimal degrees to radians
 * @param degree
 * @retval radians
 */
float deg2rad(float deg) {
	return (deg * pi / 180);
}

/**
 * @brief converts radians to decimal degrees
 * @param radians
 * @retval degrees
 */
float rad2deg(float rad) {
	return (rad * 180 / pi);
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_UART_Transmit(&huart1, "Txed", 4, 100);
  //Xfer_Complete = 1;
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_UART_Transmit(&huart1, aRxBuffer, 16, 100);

  char *s, *t, *d, *l;
  s = strstr(aRxBuffer, "Linestat");
  t = strstr(aRxBuffer, "SetLocation");
  d = strstr(aRxBuffer, "GetTime");
  l = strstr(aRxBuffer, "SetLineState");
  if (s != NULL){
	  char * txM;
	  memset(aTxBuffer, '\0', 16);
	  switch (linesInd) {
	  case 0:
		  txM = "LineStatus STOP.";
		  break;
	  case 1:
		  txM = "LineStatus RUN..";
		  break;
	  case 2:
		  txM = "LineStatus FASTF";
		  break;
	  case 3:
		  txM = "LineStatus FF>12";
		  break;
	  }
	  sprintf(aTxBuffer, txM);
	  HAL_UART_Transmit(&huart1, aTxBuffer, 16, 100);
	  HAL_UART_Transmit(&huart1, "It's here", 9, 100);
  }
  if (t != NULL) {
	  setLoc = 1;
	  memset(aTxBuffer, '\0', 16);
	  uint8_t txM = "Setting Location";
	  sprintf(aTxBuffer, txM);
  }
  if (d != NULL) {
	  memset(aTxBuffer, '\0', 16);
	  sprintf(aTxBuffer, timeBuffer + "....");
	  HAL_UART_Transmit(&huart1, aTxBuffer, 16, 100);
	  HAL_UART_Transmit(&huart1, "It's not here", 12, 100);
  }
  if (l != NULL) {
	  uint8_t st;
	  char *c, *txM;
	  sscanf(aRxBuffer, "%s %d", c, st);
	  switch (st) {
	  case 0:
		  txM = "Line Set to STOP";
		  break;
	  case 1:
		  txM = "Line Set to RUN.";
		  break;
	  case 2:
		  txM = "Line FForward>12";
		  break;
	  }
	  memset(aTxBuffer, '\0', 16);
	  sprintf(aTxBuffer, txM);
	  HAL_UART_Transmit(&huart1, aTxBuffer, 16, 100);
	  HAL_UART_Transmit(&huart1, "It's not here", 12, 100);
  }
}

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	Transfer_Direction = TransferDirection;
	//if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
	if (Transfer_Direction != I2C_DIRECTION_TRANSMIT) {
		//HAL_UART_Transmit(&huart1, "TXX", 3, 100);
		/*##- Put I2C peripheral in reception process ###########################*/
		if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t *)aTxBuffer, 16, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
		{
			/* Transfer error in reception process */
			Error_Handler();
		}
	}
	else {
		//HAL_UART_Transmit(&huart1, "RXX", 3, 100);
		/*##- Put I2C peripheral in reception process ###########################*/
		if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t *)aRxBuffer, 16, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
		{
			/* Transfer error in reception process */
			Error_Handler();
		}
		//HAL_UART_Transmit(&huart1, "RXC", 3, 100);
	}

}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	Xfer_Complete = 1;
	HAL_UART_Transmit(&huart1, "Lcpl", 4, 100);
}


/**
  * @brief  I2C error callbacks
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /** Error_Handler() function is called when error occurs.
  * 1- When Slave don't acknowledge it's address, Master restarts communication.
  * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
  */
  if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
  {
    Error_Handler();
  }
}

/**
 * @brief Timer Interrupt callback
 * @param Timer Handle
 * @retval none
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (secS < 60) {
		secS++;
	} else {
		secS = 0;
		minP = 1;
	}
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
