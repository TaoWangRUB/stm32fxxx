/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
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
/* USER CODE BEGIN Variables */
static cnt = 0;
/* USER CODE END Variables */
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for lcdTask */
osThreadId_t lcdTaskHandle;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for mpu6050Task */
osThreadId_t mpu6050TaskHandle;
const osThreadAttr_t mpu6050Task_attributes = {
  .name = "mpu6050Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for icm20948Task */
osThreadId_t icm20948TaskHandle;
const osThreadAttr_t icm20948Task_attributes = {
  .name = "icm20948Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskKalmanFilte */
osThreadId_t taskKalmanFilteHandle;
const osThreadAttr_t taskKalmanFilte_attributes = {
  .name = "taskKalmanFilte",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bmp280Task */
osThreadId_t bmp280TaskHandle;
const osThreadAttr_t bmp280Task_attributes = {
  .name = "bmp280Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mutexMpu6050 */
osMutexId_t mutexMpu6050Handle;
const osMutexAttr_t mutexMpu6050_attributes = {
  .name = "mutexMpu6050"
};
/* Definitions for mutexIcm20948 */
osMutexId_t mutexIcm20948Handle;
const osMutexAttr_t mutexIcm20948_attributes = {
  .name = "mutexIcm20948"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartButtonTask(void *argument);
void StartLcdTask(void *argument);
void StartTaskMpu6050(void *argument);
void StartTaskIcm20948(void *argument);
void StartTaskKalmanFilter(void *argument);
void StartTaskBmp280(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of mutexMpu6050 */
  mutexMpu6050Handle = osMutexNew(&mutexMpu6050_attributes);

  /* creation of mutexIcm20948 */
  mutexIcm20948Handle = osMutexNew(&mutexIcm20948_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  dmaCompleteSemaphore = xSemaphoreCreateBinary();
  if (dmaCompleteSemaphore == NULL) {
	  // Handle semaphore creation failure
	  printf("Failed to create dmaCompleteSemaphore.\r\n");
	  while (1);
  }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);

  /* creation of lcdTask */
  lcdTaskHandle = osThreadNew(StartLcdTask, NULL, &lcdTask_attributes);

  /* creation of mpu6050Task */
  mpu6050TaskHandle = osThreadNew(StartTaskMpu6050, NULL, &mpu6050Task_attributes);

  /* creation of icm20948Task */
  icm20948TaskHandle = osThreadNew(StartTaskIcm20948, NULL, &icm20948Task_attributes);

  /* creation of taskKalmanFilte */
  taskKalmanFilteHandle = osThreadNew(StartTaskKalmanFilter, NULL, &taskKalmanFilte_attributes);

  /* creation of bmp280Task */
  bmp280TaskHandle = osThreadNew(StartTaskBmp280, NULL, &bmp280Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartButtonTask */
/**
  * @brief  Function implementing the buttonTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  /* Infinite loop */
	uint32_t now;
	for(;;)
	{
	  if(board_button_pressed())
	  {
		  board_led_set(GPIO_PIN_SET);
	  }
	  else
	  {
		  if(now >= 100)
		  {
			  now = 0;
			  board_led_toggle();
		  }
		  else
		  {
			  now++;
		  }
	  }
	  osDelay(10);
	}
  /* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartLcdTask */
/**
* @brief Function implementing the lcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLcdTask */
void StartLcdTask(void *argument)
{
  /* USER CODE BEGIN StartLcdTask */
  /* Infinite loop */
	for(;;)
	{
	  if(1)
	  {
		  Display_Accel_Data();
	  }
	  // printf("%d, %.5f, %.5f, %.5f, ", ++cnt, MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
	  // printf("%.5f, %.5f, %.5f\r\n", ICM20948.acce[0], ICM20948.acce[1], ICM20948.acce[2]);
	  //printf(" Mx: %.5f My: %.5f Mz: %.5f\r\n", ICM20948.mage[0], ICM20948.mage[1], ICM20948.mage[2]);
	  printf("Temp: %.5f, Pressure: %.5f\r\n", BMP280.temp, BMP280.press);
	  osDelay(50);
	}
  /* USER CODE END StartLcdTask */
}

/* USER CODE BEGIN Header_StartTaskMpu6050 */
/**
* @brief Function implementing the mpu6050Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMpu6050 */
void StartTaskMpu6050(void *argument)
{
  /* USER CODE BEGIN StartTaskMpu6050 */
  /* Infinite loop */
	  for(;;)
	  {
		  current_i2c_dma_state = I2C_DMA_STATE_MPU6050;
		  MPU6050_Read_DMA(&hi2c1, &MPU6050);
		  if (xSemaphoreTake(dmaCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
			  // Process MPU6050 data
			  MPU6050_Process_Data(&MPU6050);
			  //printf("MPU6050 Ax: %d Ay: %d Az: %d\r\n", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
		  }
		  osDelay(10);
	  }
  /* USER CODE END StartTaskMpu6050 */
}

/* USER CODE BEGIN Header_StartTaskIcm20948 */
/**
* @brief Function implementing the icm20948Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskIcm20948 */
void StartTaskIcm20948(void *argument)
{
  /* USER CODE BEGIN StartTaskIcm20948 */
  /* Infinite loop */
	  for(;;)
	  {
		  // first read acc & gyro
		  current_i2c_dma_state = I2C_DMA_STATE_ICM20948_ACCEL_GYRO;
		  ICM20948_ReadDMA(&ICM20948);
		  if (xSemaphoreTake(dmaCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
			  ICM20948_Process_Gyro_data(&ICM20948);
		      //printf("ICM20948 Ax: %d Ay: %d Az: %d\r\n", ax, ay, az);
		  }
		  // then read mag
		  current_i2c_dma_state = I2C_DMA_STATE_ICM20948_MAG;
		  Magnetometer_ReadDMA(&ICM20948);
		  if (xSemaphoreTake(dmaCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
			  ICM20948_Process_Mage_data(&ICM20948);
		      //printf("ICM20948 Mx: %d My: %d Mz: %d\r\n", ICM20948.mage[0], ICM20948.mage[1], ICM20948.mage[2]);
		  }
		  osDelay(10);
	  }
  /* USER CODE END StartTaskIcm20948 */
}

/* USER CODE BEGIN Header_StartTaskKalmanFilter */
/**
* @brief Function implementing the taskKalmanFilte thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskKalmanFilter */
void StartTaskKalmanFilter(void *argument)
{
  /* USER CODE BEGIN StartTaskKalmanFilter */
  /* Infinite loop */
	  for(;;)
	  {
		osDelay(10);
	  }
  /* USER CODE END StartTaskKalmanFilter */
}

/* USER CODE BEGIN Header_StartTaskBmp280 */
/**
* @brief Function implementing the bmp280Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskBmp280 */
void StartTaskBmp280(void *argument)
{
  /* USER CODE BEGIN StartTaskBmp280 */
  /* Infinite loop */
	for(;;)
	{
		current_i2c_dma_state = I2C_DMA_STATE_BMP280;
		//BMP280_ReadTempAndPressure(&BMP280);
		BMP280_Read_DMA(&BMP280);
		if (xSemaphoreTake(dmaCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
			BMP280_Process_data(&BMP280);
		}
		osDelay(10);
	}
  /* USER CODE END StartTaskBmp280 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

