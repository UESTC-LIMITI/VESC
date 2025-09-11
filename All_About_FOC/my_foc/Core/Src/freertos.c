/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_interface.h"
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for foc_pid */
osThreadId_t foc_pidHandle;
const osThreadAttr_t foc_pid_attributes = {
  .name = "foc_pid",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for foc_sensor */
osThreadId_t foc_sensorHandle;
const osThreadAttr_t foc_sensor_attributes = {
  .name = "foc_sensor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for foc_comm */
osThreadId_t foc_commHandle;
const osThreadAttr_t foc_comm_attributes = {
  .name = "foc_comm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void foc_pid_thread(void *argument);
void foc_sensor_thread(void *argument);
void foc_comm_thread(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of foc_pid */
  foc_pidHandle = osThreadNew(foc_pid_thread, NULL, &foc_pid_attributes);

  /* creation of foc_sensor */
  foc_sensorHandle = osThreadNew(foc_sensor_thread, NULL, &foc_sensor_attributes);

  /* creation of foc_comm */
  foc_commHandle = osThreadNew(foc_comm_thread, NULL, &foc_comm_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_foc_pid_thread */
/**
* @brief Function implementing the foc_pid thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_foc_pid_thread */
void foc_pid_thread(void *argument)
{
  /* USER CODE BEGIN foc_pid_thread */
  /* Infinite loop */
  for(;;)
  {
    interface_run_pid();
    osDelay(1);
  }
  /* USER CODE END foc_pid_thread */
}

/* USER CODE BEGIN Header_foc_sensor_thread */
/**
* @brief Function implementing the foc_sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_foc_sensor_thread */
void foc_sensor_thread(void *argument)
{
  /* USER CODE BEGIN foc_sensor_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END foc_sensor_thread */
}

/* USER CODE BEGIN Header_foc_comm_thread */
/**
* @brief Function implementing the foc_comm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_foc_comm_thread */
void foc_comm_thread(void *argument)
{
  /* USER CODE BEGIN foc_comm_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END foc_comm_thread */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

