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
#include <string.h>
#include "usart.h"
#include "i2c.h"
#include "bmp280.h"
#include "BH1750.h"
#include <stdio.h>
#include "mpu6050.h"
#include "moving-median.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Define a structure to represent sensor readings
typedef struct {
    float value;
    float standard_deviation;
    float median;
    float min;
    float max;
} SensorStats;

typedef struct {
	SensorStats temperature;
	SensorStats pressure;
} Sensor_BMP280;

typedef struct {
	SensorStats gyro_x;
	SensorStats gyro_y;
	SensorStats gyro_z;
} Sensor_MPU6050;

typedef struct {
	SensorStats lumen;
} Sensor_BH1750;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
movingMedian_t med_filter;

/* USER CODE END Variables */
/* Definitions for sensorProduce */
osThreadId_t sensorProduceHandle;
const osThreadAttr_t sensorProduce_attributes = {
  .name = "sensorProduce",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BLConsumer */
osThreadId_t BLConsumerHandle;
const osThreadAttr_t BLConsumer_attributes = {
  .name = "BLConsumer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void sensor_data_producer(void *argument);
void bl_data_consumer(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sensorProduce */
  sensorProduceHandle = osThreadNew(sensor_data_producer, NULL, &sensorProduce_attributes);

  /* creation of BLConsumer */
  BLConsumerHandle = osThreadNew(bl_data_consumer, NULL, &BLConsumer_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_sensor_data_producer */
/**
  * @brief  Function implementing the sensorProduce thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_sensor_data_producer */
void sensor_data_producer(void *argument)
{
  /* USER CODE BEGIN sensor_data_producer */
  uint16_t size;
  uint8_t Data[256];
  moving_median_create(&med_filter, 11, 100);
  /////-------BMP280 init--------////
  BMP280_HandleTypedef bmp280;
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  float pressure, temperature, humidity;
  while (!bmp280_init(&bmp280, &bmp280.params));
  bool bme280p = bmp280.id == BME280_CHIP_ID;
  /////-------BMP280 init--------/////

  /////-------BH1750 init--------/////
  BH1750_init_i2c(&hi2c1);
  BH1750_device_t* test_dev = BH1750_init_dev_struct(&hi2c1, "test device", true);
  BH1750_init_dev(test_dev);
  /////-------BH1750 init--------/////

  ////-------MPU6050 init-------/////
  MPU6050_t MPU6050;
  while (MPU6050_Init(&hi2c1) == 1);
  ////-------MPU6050 init-------/////


  /* Infinite loop */
  for(;;)
  {
	///--------------BMP280-----------------////
	//osDelay(100);;
	while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity));

	size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C \r\n", pressure, temperature);
	HAL_UART_Transmit(&huart1, Data, size, 1000);

	if (bme280p) {
		size = sprintf((char *)Data,", Humidity: %.2f\r\n", humidity);
		HAL_UART_Transmit(&huart1, Data, size, 1000);
	}
	//osDelay(100);
	///--------------BMP280-----------------///

	/////------------BH1750--------/////
    test_dev->poll(test_dev);
    Sensor_BH1750 sensor_bh1750;
    sensor_bh1750.lumen.value = test_dev->value;
    size = sprintf((char *)Data,", Lumen: %.2f \r\n", sensor_bh1750.lumen.value);
    HAL_UART_Transmit(&huart1, Data, size, 1000);
    //osDelay(100);
    moving_median_filter(&med_filter, sensor_bh1750.lumen.value);
    size = sprintf((char *)Data,", Filtered Lumen: %d \r\n", med_filter.filtered);
    HAL_UART_Transmit(&huart1, Data, size, 1000);
	//osDelay(100);
	/////-------BH1750--------/////

	////-------MPU6050-------/////
	MPU6050_Read_Gyro(&hi2c1, &MPU6050);
	size = sprintf((char *)Data,", GyroX: %.2f  GyroY: %.2f  GyroZ: %.2f \r\n", MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
	HAL_UART_Transmit(&huart1, Data, size, 1000);
	////-------MPU6050-------/////

	//osDelay(100);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  osThreadTerminate(NULL);
  /* USER CODE END sensor_data_producer */
}

/* USER CODE BEGIN Header_bl_data_consumer */
/**
* @brief Function implementing the BLConsumer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bl_data_consumer */
void bl_data_consumer(void *argument)
{
  /* USER CODE BEGIN bl_data_consumer */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  osThreadTerminate(NULL);
  /* USER CODE END bl_data_consumer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

