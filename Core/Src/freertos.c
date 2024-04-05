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
#include <stdlib.h>
#include "usart.h"
#include "i2c.h"
#include "bmp280.h"
#include "BH1750.h"
#include <stdio.h>
#include "mpu6050.h"
#include "moving-median.h"
#include "circular_buffer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Define a structure to represent sensor readings


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
movingMedian_t med_filter_temperature;
movingMedian_t med_filter_lumen;
movingMedian_t med_filter_gyro_x;
movingMedian_t med_filter_gyro_y;
movingMedian_t med_filter_gyro_z;

static SensorData circular_buffer_storage_[CIRCULAR_BUFFER_SIZE] = {0};
static cbuf_handle_t cbuf_handle = NULL;

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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
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
	cbuf_handle = circular_buf_init(circular_buffer_storage_, CIRCULAR_BUFFER_SIZE);

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
  //uint16_t size;
  //uint8_t Data[256];

  //uint8_t * buffer  = malloc(sizeof(Data));
  //cbuf_handle_t me = circular_buf_init(buffer, sizeof(Data));

  moving_median_create(&med_filter_temperature, 5, 50);
  moving_median_create(&med_filter_lumen, 5, 50);
  moving_median_create(&med_filter_gyro_x, 5, 50);
  moving_median_create(&med_filter_gyro_y, 5, 50);
  moving_median_create(&med_filter_gyro_z, 5, 50);
  /////-------BMP280 init--------////
  BMP280_HandleTypedef bmp280;
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  while (!bmp280_init(&bmp280, &bmp280.params));

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
  SensorData sensor_data;

  /* Infinite loop */
  for(;;)
  {
	///--------------BMP280-----------------////

	//get sensor value
	while (!bmp280_read_float_temperature(&bmp280, &sensor_data.sensor_bmp280.temperature.value));

	//size = sprintf((char *)Data,"Temperature: %.2f C \r\n", sensor_data.sensor_bmp280.temperature.value);
	//HAL_UART_Transmit(&huart1, Data, size, 1000);

	//apply filter
	moving_median_filter(&med_filter_temperature, sensor_data.sensor_bmp280.temperature.value);
	sensor_data.sensor_bmp280.temperature.median = med_filter_temperature.filtered;

	//size = sprintf((char *)Data,"Filtered Temperature: %.2f C \r\n", sensor_data.sensor_bmp280.temperature.median);
	//HAL_UART_Transmit(&huart1, Data, size, 1000);
	///--------------BMP280-----------------///

	/////------------BH1750--------/////
	//get sensor value
    test_dev->poll(test_dev);
    sensor_data.sensor_bh1750.lumen.value = test_dev->value;
    //size = sprintf((char *)Data,", Lumen: %.2f \r\n", sensor_data.sensor_bh1750.lumen.value);
    //HAL_UART_Transmit(&huart1, Data, size, 1000);

    //apply filter
    moving_median_filter(&med_filter_lumen, sensor_data.sensor_bh1750.lumen.value);
    //osDelay(10);
    sensor_data.sensor_bh1750.lumen.median = med_filter_lumen.filtered;
    //size = sprintf((char *)Data,", Get from producer Filtered Lumen: %.2f \r\n", sensor_data.sensor_bh1750.lumen.median);
    //HAL_UART_Transmit(&huart1, Data, size, 1000);
	//osDelay(100);
	/////-------BH1750--------/////

	////-------MPU6050-------/////
	MPU6050_Read_Gyro(&hi2c1, &MPU6050);
	sensor_data.sensor_mpu6050.gyro_x.value = MPU6050.Gx;
	sensor_data.sensor_mpu6050.gyro_y.value = MPU6050.Gy;
	sensor_data.sensor_mpu6050.gyro_z.value = MPU6050.Gz;
	//size = sprintf((char *)Data,", GyroX: %.2f  GyroY: %.2f  GyroZ: %.2f \r\n", sensor_data.sensor_mpu6050.gyro_x.value,
	//		sensor_data.sensor_mpu6050.gyro_y.value, sensor_data.sensor_mpu6050.gyro_z.value);
	//HAL_UART_Transmit(&huart1, Data, size, 1000);

	moving_median_filter(&med_filter_gyro_x, sensor_data.sensor_mpu6050.gyro_x.value);
	sensor_data.sensor_mpu6050.gyro_x.median = med_filter_gyro_x.filtered;
	osDelay(10);
	moving_median_filter(&med_filter_gyro_y, sensor_data.sensor_mpu6050.gyro_y.value);
	sensor_data.sensor_mpu6050.gyro_y.median = med_filter_gyro_y.filtered;
	osDelay(10);
	moving_median_filter(&med_filter_gyro_z, sensor_data.sensor_mpu6050.gyro_z.value);
	sensor_data.sensor_mpu6050.gyro_z.median = med_filter_gyro_z.filtered;
	osDelay(10);
	//size = sprintf((char *)Data,", FILTERED GyroX: %.2f  GyroY: %.2f  GyroZ: %.2f \r\n", sensor_data.sensor_mpu6050.gyro_x.median,
	//		sensor_data.sensor_mpu6050.gyro_y.median, sensor_data.sensor_mpu6050.gyro_z.median);
	//HAL_UART_Transmit(&huart1, Data, size, 1000);
	////-------MPU6050-------/////
	circular_buf_put(cbuf_handle, sensor_data);

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
	SensorData broadcast_sensor_data;
	//uint8_t a;
	uint8_t size;
	uint8_t Data[250];

  /* Infinite loop */
  for(;;)
  {
	circular_buf_get(cbuf_handle, &broadcast_sensor_data);
	osDelay(10);
	size = sprintf((char *)Data,"Lumen: Value %.2f  Median: %.2f \n "
			"Temperature: Value %.2f  Median: %.2f \n "
			"Gyro: Value X %.2f Y %.2f Z %.2f \n"
			"Gyro: Median X %.2f Y %.2f Z %.2f \n",
			broadcast_sensor_data.sensor_bh1750.lumen.value,
			broadcast_sensor_data.sensor_bh1750.lumen.median,
			broadcast_sensor_data.sensor_bmp280.temperature.value,
			broadcast_sensor_data.sensor_bmp280.temperature.median,
			broadcast_sensor_data.sensor_mpu6050.gyro_x.value,
			broadcast_sensor_data.sensor_mpu6050.gyro_y.value,
			broadcast_sensor_data.sensor_mpu6050.gyro_z.value,
			broadcast_sensor_data.sensor_mpu6050.gyro_x.median,
			broadcast_sensor_data.sensor_mpu6050.gyro_y.median,
			broadcast_sensor_data.sensor_mpu6050.gyro_z.median);
	osDelay(10);
	HAL_UART_Transmit(&huart1, Data, size, 1000);
    osDelay(1000);
  }
  osThreadTerminate(NULL);
  /* USER CODE END bl_data_consumer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

