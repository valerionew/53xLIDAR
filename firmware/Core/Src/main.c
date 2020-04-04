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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l1_api.h"

#include "WS2812.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Gpio {
	GPIO_TypeDef * const port;
	const uint16_t mask;
};

struct Gpio leds = { GPIOA, GPIO_PIN_15 };

struct Gpio xshut[] = { { GPIOC, GPIO_PIN_0 }, { GPIOC, GPIO_PIN_1 }, { GPIOC,
GPIO_PIN_2 }, { GPIOC, GPIO_PIN_3 }, { GPIOA, GPIO_PIN_4 }, { GPIOA,
GPIO_PIN_5 }, { GPIOA, GPIO_PIN_6 }, { GPIOA, GPIO_PIN_7 }, };
/*
 uint8_t interruptLidar[] = {
 PC4, PC5, PB0, PB1, PB2, PB14, PB15, PC6, PC7, PC8, PC15, PC13, PC14, PB9, PA0, PA1
 };
 */

struct {
	VL53L1_Dev_t chip;
	uint8_t valid;
} sensors[16];

int enableSensor(int i) {

	sensors[i].chip.I2cDevAddr = 0x29 << 1;
	if (i < 8) {
		sensors[i].chip.I2cHandle = &hi2c1;
	} else {
		sensors[i].chip.I2cHandle = &hi2c2;
	}

	sensors[i].valid = 0;

	char buff[15];
	sprintf(buff, "sensor: %d\n", i);
	HAL_UART_Transmit(&huart1, buff, strlen(buff), 0xFFFF);

	/*** VL53L1X Initialization ***/
	VL53L1_Error err;
	err = VL53L1_WaitDeviceBooted(&sensors[i].chip);
	if (err != VL53L1_ERROR_NONE) {
		uint8_t msg[] = "VL53L1_WaitDeviceBooted failed    \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		msg[len - 3 - 1] = err + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
		return 1;
	} else {
		uint8_t msg[] = "VL53L1_WaitDeviceBooted OK  \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
	}

	err = VL53L1_DataInit(&sensors[i].chip);
	if (err != VL53L1_ERROR_NONE) {
		uint8_t msg[] = "VL53L1_DataInit failed    \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		msg[len - 3 - 1] = err + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
		return 3;
	} else {
		uint8_t msg[] = "VL53L1_DataInit OK  \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
	}

	err = VL53L1_SetDeviceAddress(&sensors[i].chip, (0x29 + i + 1) << 1);
	sensors[i].chip.I2cDevAddr = (0x29 + i + 1) << 1; //change address even in case of error to reduce miss-talk
	if (err != VL53L1_ERROR_NONE) {
		uint8_t msg[] = "VL53L1_SetDeviceAddress failed    \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		msg[len - 3 - 1] = err + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
		return 2;
	} else {
		uint8_t msg[] = "VL53L1_SetDeviceAddress OK  \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
	}

	err = VL53L1_StaticInit(&sensors[i].chip);
	if (err != VL53L1_ERROR_NONE) {
		uint8_t msg[] = "VL53L1_StaticInit failed    \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		msg[len - 3 - 1] = err + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
		return 4;
	} else {
		uint8_t msg[] = "VL53L1_StaticInit OK  \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
	}

	err = VL53L1_SetDistanceMode(&sensors[i].chip, VL53L1_DISTANCEMODE_LONG);
	if (err != VL53L1_ERROR_NONE) {
		uint8_t msg[] = "VL53L1_SetDistanceMode failed    \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		msg[len - 3 - 1] = err + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
		return 5;
	} else {
		uint8_t msg[] = "VL53L1_SetDistanceMode OK  \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
	}

	err = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&sensors[i].chip,
			20000);
	if (err != VL53L1_ERROR_NONE) {
		uint8_t msg[] =
				"VL53L1_SetMeasurementTimingBudgetMicroSeconds failed    \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		msg[len - 3 - 1] = err + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
		return 6;
	} else {
		uint8_t msg[] = "VL53L1_SetMeasurementTimingBudgetMicroSeconds OK  \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
	}

	err = VL53L1_SetInterMeasurementPeriodMilliSeconds(&sensors[i].chip, 25);
	if (err != VL53L1_ERROR_NONE) {
		uint8_t msg[] =
				"VL53L1_SetInterMeasurementPeriodMilliSeconds failed    \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		msg[len - 3 - 1] = err + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
		return 7;
	} else {
		uint8_t msg[] = "VL53L1_SetInterMeasurementPeriodMilliSeconds OK  \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
	}

	err = VL53L1_StartMeasurement(&sensors[i].chip);
	if (err != VL53L1_ERROR_NONE) {
		uint8_t msg[] = "VL53L1_StartMeasurement failed    \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		msg[len - 3 - 1] = err + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
		return 8;
	} else {
		uint8_t msg[] = "VL53L1_StartMeasurement OK  \n";
		uint8_t len = sizeof(msg) - 1;
		msg[len - 1 - 1] = i + '0';
		HAL_UART_Transmit(&huart1, msg, len, 0xFFFF);
	}
	sensors[i].valid = 1;
	return 0;
}

void scan(I2C_HandleTypeDef *hi2c) {
	for (int i = 1; i < 128; i++) {
		/*
		 * the HAL wants a left aligned i2c address
		 * &hi2c1 is the handle
		 * (uint16_t)(i<<1) is the i2c address left aligned
		 * retries 2
		 * timeout 2
		 */
		HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(hi2c, (i << 1), 2, 2);
		if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
				{
			HAL_UART_Transmit(&huart1, ".", 1, 0xFFFF);
		}
		if (result == HAL_OK) {
			char buff[80];
			sprintf(buff, "0x%X\n", i << 1);
			HAL_UART_Transmit(&huart1, buff, strlen(buff), 0xFFFF);
		}
	}
	char buff[] = "\nSCAN FINISH\n";
	HAL_UART_Transmit(&huart1, buff, strlen(buff), 0xFFFF);
}

uint32_t ledColor[4];

void updateLed() {
	static int lastGreen = 0;
	static uint8_t first = 1;
	static uint32_t last = 0;
	if (!first) {
		// set back previus led
		if (lastGreen == 0) {
			ledColor[3] = last;
		} else {
			ledColor[lastGreen - 1] = last;
		}
	} else {
		first = 0;
	}
	//set current to blue and increment
	last = ledColor[lastGreen];
	ledColor[lastGreen] = 0x00FF00; //blue
	lastGreen++;

	//avoid overflow
	if (lastGreen >= 4) {
		lastGreen = 0;
	}

	sendColors(ledColor, 4);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	// be sure to put all xshut to low
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
			GPIO_PIN_RESET);

	for (int i = 0; i < 4; i++) {
		ledColor[i] = 0x0000FF; //set to blue
	}
	sendColors(ledColor, 4);

	HAL_Delay(1000);

	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(xshut[i].port, xshut[i].mask, GPIO_PIN_SET);
		HAL_Delay(10);
		int error = 0;
		error += enableSensor(i); //i2c1
		error += enableSensor(i + 8); //i2c2

		if (error) {
			//HAL_GPIO_WritePin(xshut[i].port, xshut[i].mask, GPIO_PIN_RESET);
			ledColor[i / 2] = 0xFF0000; //set to red
			sendColors(ledColor, 4);
		} else {
			if (ledColor[i / 2] != 0xFF0000) {
				ledColor[i / 2] = 0; //turn off led
				sendColors(ledColor, 4);
			}
		}
	}
	scan(&hi2c1);
	scan(&hi2c2);

	int fps = 0;
	uint8_t buff[80];

	uint8_t msg[5]; // ~, 3 byte payload, checksum
	uint32_t start = HAL_GetTick();
	while (1) {

		for (int i = 0; i < 16; i++) {

			VL53L1_RangingMeasurementData_t rangingData;

			uint8_t ready = 0;
			if (sensors[i].valid) {
				VL53L1_Error err = VL53L1_GetMeasurementDataReady(
						&sensors[i].chip, &ready);
				if (!err && ready) {

					VL53L1_GetRangingMeasurementData(&sensors[i].chip,
							&rangingData);

					VL53L1_ClearInterruptAndStartMeasurement(&sensors[i].chip);

					msg[0] = '~';
					msg[1] = i;
					msg[2] = rangingData.RangeMilliMeter;
					msg[3] = rangingData.RangeMilliMeter >> 8;
					msg[4] = msg[0] + msg[1] + msg[2] + msg[3];

					//HAL_UART_Transmit(&huart1, msg, sizeof(msg), 0xFFFF);

					sprintf((char*) buff, "%x\t%d\n", i,
							rangingData.RangeMilliMeter);
					HAL_UART_Transmit(&huart1, buff, strlen((char*) buff),
							0xFFFF);

					fps++;
				} else if (err) {
					sprintf((char*) buff, "error %d reading sensor %d\n", err, i);
					HAL_UART_Transmit(&huart1, buff, strlen((char*) buff), 0xFFFF);
					/*
					 maxErr--;
					 if (!maxErr){
					 sprintf((char*) buff, "reset\n");
					 HAL_UART_Transmit(&huart1, buff, strlen((char*) buff), 0xFFFF);
					 scan(&hi2c1);
					 MX_I2C1_Init();
					 scan(&hi2c1);
					 }
					 */
				} else {
					//sprintf((char*) buff, "not ready\n");0
					//HAL_UART_Transmit(&huart1, buff, strlen((char*) buff), 0xFFFF);
				}
			}
		}

		static uint32_t startLedSpin = 0;
		if (HAL_GetTick() - startLedSpin >= 100) {
			startLedSpin = HAL_GetTick();
			updateLed();
		}

		if (HAL_GetTick() - start >= 1000) {
			start = HAL_GetTick();
			sprintf((char*) buff, "FPS: %d\n", fps);
			HAL_UART_Transmit(&huart1, buff, strlen((char*) buff), 0xFFFF);
			fps = 0;
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC4 
                           PC5 PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB14 
                           PB15 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
