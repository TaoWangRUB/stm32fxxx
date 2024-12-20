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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board.h"
#include <stdio.h>
//#include "MPU6050/mpu6050.h"
#include "IMU10DOF/Waveshare_10Dof-D.h"
#include "OLED/ssd1306.h"
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

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
ICM20948_t ICM20948;
BMP280_t BMP280;
volatile I2C_DMA_State current_i2c_dma_state = I2C_DMA_STATE_NONE;
SemaphoreHandle_t dmaCompleteSemaphore;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Scan(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Redirect printf to UART */
#ifdef __GNUC__
  // For GCC, implement __io_putchar for printf redirection
  int __io_putchar(int ch)
#else
  // For other compilers, implement fputc for printf redirection
  int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
    // Transmit the character over UART1
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  I2C_Scan();
  while (MPU6050_Init(&hi2c1, &MPU6050) == 1);

  if(ICM20948_Init(&ICM20948) != HAL_OK)
  {
	  printf("init icm20948 fails\r\n");
  }

  if(BMP280_Init(&BMP280) != HAL_OK)
  {
	  printf("init bmp280 fails\r\n");
  }

  ssd1306_Init();
  ssd1306_WriteString("Hello World", Font_7x10);
  ssd1306_UpdateScreen();

/*
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

  imuInit(&enMotionSensorType, &enPressureType);
  if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
  {
	  printf("Motion sersor is ICM-20948\r\n" );
  }
  else
  {
	  printf("Motion sersor NULL\r\n");
  }
  if(IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
  {
	  printf("Pressure sersor is BMP280\r\n");
  }
  else
  {
	  printf("Pressure sersor NULL\r\n");
  }
*/
  board_button_init();
  board_led_init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void I2C_Scan(void)
{

	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructureget;
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
    printf("Scanning I2C bus on Y20%02d.M%02d.D%02d at %02d:%02d:%02d...\r\n",
    		sdatestructureget.Year,
			sdatestructureget.Month,
			sdatestructureget.Date,
			stimestructureget.Hours,
			stimestructureget.Minutes,
			stimestructureget.Seconds);
    for (uint16_t addr = 0; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            printf("Device found at 0x%02X\r\n", addr);
        }
    }
    printf("Scanning I2C DONE!...\r\n");
}

void Display_Accel_Data(void) {
    // Clear the screen
    ssd1306_Clear();
    char buffer[20];  // Buffer to hold the text
    uint8_t len = 75;

	// Display the Ax value
	snprintf(buffer, sizeof(buffer), "x:%8.5f|%8.5f", MPU6050.acce[0], -(ICM20948.acce[1]));
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(buffer, Font_7x10);

	// Display the Ay value
	snprintf(buffer, sizeof(buffer), "y:%8.5f|%8.5f", MPU6050.acce[1], ICM20948.acce[0]);
	ssd1306_SetCursor(0, 10);
	ssd1306_WriteString(buffer, Font_7x10);

	// Display the Az value
	snprintf(buffer, sizeof(buffer), "z:%8.5f|%8.5f", MPU6050.acce[2], ICM20948.acce[2]);
	ssd1306_SetCursor(0, 20);
	ssd1306_WriteString(buffer, Font_7x10);

	// Update the screen
	ssd1306_UpdateScreen();

}

void Display_Gyro_Data(void) {
    // Clear the screen
    ssd1306_Clear();
    char buffer[20];  // Buffer to hold the text
    uint8_t len = 75;

	// Display the Ax value
	snprintf(buffer, sizeof(buffer), "x:%8.5f|%8.5f", MPU6050.gyro[0], -ICM20948.gyro[1]);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(buffer, Font_7x10);

	// Display the Ay value
	snprintf(buffer, sizeof(buffer), "y:%8.5f|%8.5f", MPU6050.gyro[1], ICM20948.gyro[0]);
	ssd1306_SetCursor(0, 10);
	ssd1306_WriteString(buffer, Font_7x10);

	// Display the Az value
	snprintf(buffer, sizeof(buffer), "z:%8.5f|%8.5f", MPU6050.gyro[2], ICM20948.gyro[2]);
	ssd1306_SetCursor(0, 20);
	ssd1306_WriteString(buffer, Font_7x10);

	// Update the screen
	ssd1306_UpdateScreen();

}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    	xSemaphoreGiveFromISR(dmaCompleteSemaphore, &xHigherPriorityTaskWoken);
    	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    }
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
    	switch (current_i2c_dma_state)
		{
			case I2C_DMA_STATE_MPU6050:
				printf("mpu6050 i2c error");
				break;
			case I2C_DMA_STATE_ICM20948_ACCEL_GYRO:
				printf("icm20948 gyro i2c error");
				break;
			case I2C_DMA_STATE_ICM20948_MAG:
				printf("icm20948 mag i2c error");
				break;
			case I2C_DMA_STATE_BMP280:
				printf("icm20948 bmp280 i2c error");
				break;
			case I2C_DMA_STATE_NONE:
			default:
				printf("Unexpected I2C DMA state!\r\n");
				break;
		}
    }
}
/* USER CODE END 4 */

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
