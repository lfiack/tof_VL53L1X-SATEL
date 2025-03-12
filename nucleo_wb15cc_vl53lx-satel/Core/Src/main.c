/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "VL53L1X_api.h"
#include "vl53l1_error_codes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL53L1X_DEFAULT_ADDRESS (0x29 << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&huart1, (uint8_t*) &chr, 1, HAL_MAX_DELAY);
	return chr;
}

uint32_t measure_counter = 0;
uint8_t data_available = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (INT_Pin == GPIO_Pin)
	{
		measure_counter++;
		data_available++;
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

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	printf("\r\n===== VL53L1X-SATEL =====\r\n");

	HAL_GPIO_WritePin(XSDN_GPIO_Port, XSDN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	uint16_t id;
	if (VL53L1X_GetSensorId(VL53L1X_DEFAULT_ADDRESS, &id) != VL53L1_ERROR_NONE)
	{
		Error_Handler();
	}
	printf("SensorID : 0x%X\r\n", id);

	VL53L1X_Version_t version;
	if (VL53L1X_GetSWVersion(&version) != VL53L1_ERROR_NONE)
	{
		Error_Handler();
	}
	printf("Version : %u.%ub%ur%lu\r\n", version.major, version.minor, version.build, version.revision);

	uint8_t boot_state = 0;
	uint8_t boot_state_counter = 0;

	while(boot_state == 0)
	{
		boot_state_counter++;
		if (VL53L1X_BootState(VL53L1X_DEFAULT_ADDRESS, &boot_state) != VL53L1_ERROR_NONE)
		{
			Error_Handler();
		}
		HAL_Delay(1);
	}

	printf("Chip booted in %d...\r\n", boot_state_counter);

	//Loads the 135 bytes default values to initialize the sensor.
	if (VL53L1X_SensorInit(VL53L1X_DEFAULT_ADDRESS) != VL53L1_ERROR_NONE)
	{
		Error_Handler();
	}
	printf("Sensor initialized with the default values\r\n");

	if (VL53L1X_SetDistanceMode(VL53L1X_DEFAULT_ADDRESS, 1) != VL53L1_ERROR_NONE) // 1=short, limited to 1.3m
	{
		Error_Handler();
	}
	printf("Short distance mode\r\n");

	if (VL53L1X_SetTimingBudgetInMs(VL53L1X_DEFAULT_ADDRESS, 50) != VL53L1_ERROR_NONE) // in ms possible values [20, 50, 100, 200, 500]
	{
		Error_Handler();
	}

	if (VL53L1X_SetInterMeasurementInMs(VL53L1X_DEFAULT_ADDRESS, 50) != VL53L1_ERROR_NONE) // in ms, IM must be >= TB
	{
		Error_Handler();
	}

	printf("Timing budget set\r\n");

	if (VL53L1X_StartRanging(VL53L1X_DEFAULT_ADDRESS) != VL53L1_ERROR_NONE)
	{
		Error_Handler();
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (data_available > 0)
		{
			data_available = 0;
			uint16_t distance;
			if (VL53L1X_ClearInterrupt(VL53L1X_DEFAULT_ADDRESS) != VL53L1_ERROR_NONE)
			{
				Error_Handler();
			}
			if (VL53L1X_GetDistance(VL53L1X_DEFAULT_ADDRESS, &distance) != VL53L1_ERROR_NONE)
			{
				Error_Handler();
			}
			printf("distance(%lu) : %u\r\n", measure_counter, distance);
		}

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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
			|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
	PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
	PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN Smps */

	/* USER CODE END Smps */
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
