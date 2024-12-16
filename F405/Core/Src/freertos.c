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
#include "usbd_cdc_if.h"
#include "algo/angle_estimation.h"

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
  .stack_size = 512 * 4,
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
  .stack_size = 256 * 4,
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
	MX_USB_DEVICE_Init();

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
  printf("Waiting for USB...\r\n");
  osDelay(5000);
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
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xInterval = 10;
  /* Infinite loop */
	uint32_t now;
	for(;;)
	{
		TickType_t start = xTaskGetTickCount();
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
	  /*
	  TickType_t end = xTaskGetTickCount();
	  TickType_t dt = (end - start) * portTICK_PERIOD_MS;
	  if(dt > xInterval)
		  printf("Button Time: %d > %d ms\r\n", dt, xInterval);
	  */
	  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xInterval));
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
	static uint8_t display = 1;
	static uint8_t is_pressed = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xInterval = 50;
  /* Infinite loop */
	for(;;)
	{
		TickType_t start = xTaskGetTickCount();
	  if(board_button_pressed() && !is_pressed)
	  {
		  is_pressed = 1;
	  }
	  else if(!board_button_pressed() && is_pressed)
	  {
		  display = display? 0:1;
		  is_pressed = 0;
		  printf("change display\r\n");
	  }
	  switch (display)
	  {
	  	  case 1:
	  		  Display_Gyro_Data();
	  		  break;
	  	  default:
	  		  Display_Accel_Data();
	  		  break;
	  }
	  char tx_buff[128];
	  sprintf(tx_buff, "%d, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\r\n",
			  ++cnt, MPU6050.kalmanRoll[0], MPU6050.kalmanRoll[2], MPU6050.angle[0],
			  MPU6050.kalmanPitch[0], MPU6050.kalmanPitch[2], -MPU6050.angle[1],
			  (BMP280.altitude - BMP280.altitude_offset)*100);
	  CDC_Transmit_FS(tx_buff, strlen(tx_buff));
	  //printf("%.3f\r\n", cnt*0.05);
	  /*
	  printf("%d, %.5f, %.5f, %.5f, ", ++cnt, MPU6050.acce[0], MPU6050.acce[1], MPU6050.acce[2]);
	  printf("%.5f, %.5f, %.5f, ", MPU6050.gyro[0], MPU6050.gyro[1], MPU6050.gyro[2]);
	  printf("%.5f, %.5f, %.5f ", ICM20948.acce[0], ICM20948.acce[1], ICM20948.acce[2]);
	  printf("%.5f, %.5f, %.5f\r\n", ICM20948.gyro[0], ICM20948.gyro[1], ICM20948.gyro[2]);
	  */
	  //printf(" Mx: %.5f My: %.5f Mz: %.5f\r\n", ICM20948.mage[0], ICM20948.mage[1], ICM20948.mage[2]);
	  //printf("Temp: %.5f | %.5f\r\n", MPU6050.Temperature, BMP280.temp);
	  // printf("%d, %.5f, %.5f, %.5f, ", ++cnt, MPU6050.angle[0], MPU6050.angle[1], MPU6050.angle[2]);
	  // printf("%.5f, %.5f, %.5f\r\n", ICM20948.angle[1], -ICM20948.angle[0], ICM20948.angle[2]);
	  //printf("%d, %.5f, %.5f, %.5f, ", ++cnt, MPU6050.kalmanRoll[0], MPU6050.kalmanRoll[2], MPU6050.angle[0]);

	  //printf("%.5f, %.5f, %.5f\r\n", MPU6050.kalmanPitch[0], MPU6050.kalmanPitch[2], MPU6050.angle[1]);
	  TickType_t endTime = xTaskGetTickCount();  // Measure end time
	  //printf("Task Execution Time: %d ms\r\n", (endTime - startTime) * portTICK_PERIOD_MS);

	  TickType_t end = xTaskGetTickCount();
	  TickType_t dt = (end - start) * portTICK_PERIOD_MS;
	  if(dt > xInterval)
		  printf("LCD Time: %d > %d ms\r\n", dt, xInterval);
	  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xInterval));
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
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xInterval = 20;
  /* Infinite loop */
	  for(;;)
	  {
			TickType_t start = xTaskGetTickCount();
		  current_i2c_dma_state = I2C_DMA_STATE_MPU6050;
		  MPU6050_Read_DMA(&hi2c1, &MPU6050);
		  if (xSemaphoreTake(dmaCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
			  // Process MPU6050 data
			  MPU6050_Process_Data(&MPU6050);
			  //printf("MPU6050 Ax: %d Ay: %d Az: %d\r\n", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
		  }
		  /*
		  TickType_t end = xTaskGetTickCount();
		  TickType_t dt = (end - start) * portTICK_PERIOD_MS;
		  if(dt > xInterval)
			  printf("Mpu6050 Task Time: %d > %d ms\r\n", dt, xInterval);*/
		  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xInterval));
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
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xInterval = 20;
  /* Infinite loop */
	  for(;;)
	  {
			TickType_t start = xTaskGetTickCount();
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
		  /*
		  TickType_t end = xTaskGetTickCount();
		  TickType_t dt = (end - start) * portTICK_PERIOD_MS;
		  if(dt > xInterval)
			  printf("Icm20948 Time: %d > %d ms\r\n", dt, xInterval);
		  */
		  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xInterval));
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
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xInterval = 10;
  /* Infinite loop */
	  for(;;)
	  {
		  TickType_t start = xTaskGetTickCount();
		  angle_estimation(MPU6050.acce, MPU6050.angle);
		  angle_estimation(ICM20948.acce, ICM20948.angle);
		  angle_estimation_kalman(MPU6050.gyro[0], MPU6050.angle[0], MPU6050.kalmanRoll);
		  angle_estimation_kalman(MPU6050.gyro[1], -MPU6050.angle[1], MPU6050.kalmanPitch);
		  /*
		  TickType_t end = xTaskGetTickCount();
		  TickType_t dt = (end - start) * portTICK_PERIOD_MS;
		  if(dt > xInterval)
			  printf("Kalman Task Time: %d > %d ms\r\n", dt, xInterval);
		  */
		  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xInterval));
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
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xInterval = 50;
  /* Infinite loop */
	for(;;)
	{
		TickType_t start = xTaskGetTickCount();
		current_i2c_dma_state = I2C_DMA_STATE_BMP280;

		//BMP280_ReadTempAndPressure(&BMP280);

		BMP280_Read_DMA(&BMP280);
		if (xSemaphoreTake(dmaCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
			BMP280_Process_data(&BMP280);
		}

		// osDelay(10);
		/*
		TickType_t end = xTaskGetTickCount();
		TickType_t dt = (end - start) * portTICK_PERIOD_MS;
		if(dt > xInterval)
		    printf("Bmp280 Task Time: %d > %d ms\r\n", dt, xInterval);
		*/
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xInterval));
	}
  /* USER CODE END StartTaskBmp280 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

