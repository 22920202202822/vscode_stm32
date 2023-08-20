/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include "../user_Inc/Data_container.h"
#include "stdlib.h"
#include "stdio.h"
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
QueueHandle_t Remote_Queue = NULL;
QueueHandle_t Vision_Queue = NULL;
QueueHandle_t Imu_Queue = NULL;
QueueHandle_t Judgement_Queue = NULL;
osThreadId RemoteDecodeTaskHandle;
osThreadId ImuDecodeTaskHandle;
osThreadId JudgementDecodeTaskHandle;
osThreadId MotorUpdateTaskHandle;
extern remote m_remote;
extern imu m_imu;
extern judgement m_judgement;
extern void MotorUpdate(void *argument); // In Motor.cpp
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartRemoteDecode(void const * argument);
void StartImuDecode(void const * argument);
void StartJudgementDecode(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  Remote_Queue = xQueueCreate((UBaseType_t) 1,  // 1 is queue len, 18 is every data's size in queue
                          (UBaseType_t) 18);
  if(Remote_Queue == NULL)
  {
    Error_Handler();
  }
  Imu_Queue = xQueueCreate((UBaseType_t) 3,  // 1 is queue len
                          (UBaseType_t) sizeof(int));
  if(Imu_Queue == NULL)
  {
    Error_Handler();
  }
  Judgement_Queue = xQueueCreate((UBaseType_t) 1,  // 1 is queue len
                          (UBaseType_t) sizeof(int));
  if(Judgement_Queue == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  // 遥控器数据解算任务
  osThreadDef(RemoteDecodeTask, StartRemoteDecode, osPriorityHigh, 0, 128);
  RemoteDecodeTaskHandle = osThreadCreate(osThread(RemoteDecodeTask), NULL);
  // 陀螺仪数据解算任务
  osThreadDef(ImuDecodeTask, StartImuDecode, osPriorityHigh, 0, 128);
  ImuDecodeTaskHandle = osThreadCreate(osThread(ImuDecodeTask), NULL);
  // 裁判系统串口数据解算任务
  osThreadDef(JudgementDecodeTask, StartJudgementDecode, osPriorityHigh, 0, 128);
  ImuDecodeTaskHandle = osThreadCreate(osThread(JudgementDecodeTask), NULL);
  // 电机状态更新任务
  osThreadDef(MotorUpdateTask, MotorUpdate, osPriorityHigh, 0, 128);
  MotorUpdateTaskHandle = osThreadCreate(osThread(MotorUpdateTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartRemoteDecode(void const * argument)
{
  /* USER CODE BEGIN StartRemoteDecode */
  const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 20 );
  uint8_t remote_receive_buf[30];
  /* Infinite loop */
  for(;;)
  {
    if(xQueueReceive(Remote_Queue, remote_receive_buf, xMaxExpectedBlockTime) == pdPASS)
    {
      Remote_Decode(remote_receive_buf);
    }
    else
    {
      continue;
    }
  }
  /* USER CODE END StartRemoteDecode */
}
void StartImuDecode(void const * argument)
{
  /* USER CODE BEGIN StartImuDecode */
  const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 20 );
  int address;
  /* Infinite loop */
  for(;;)
  {
    if(xQueueReceive(Imu_Queue, (void *)&address, xMaxExpectedBlockTime) == pdPASS)
    {
      queue_data_t *queue_data;
      queue_data = (queue_data_t *)address;
      UartPrintf(UART4_print, "NIHAO");
      Imu_Decode(queue_data->data);
      free(queue_data);
      // if(queue_data_ptr != NULL)
      // {
      //   free(queue_data_ptr);
      //   queue_data_ptr = NULL;
      // }
    }
    else
    {
      continue;
    }
  }
  /* USER CODE END StartRemoteDecode */
}
void StartJudgementDecode(void const * argument)
{
  /* USER CODE BEGIN StartImuDecode */
  const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 20 );
  int address;
  /* Infinite loop */
  for(;;)
  {
    if(xQueueReceive(Imu_Queue, (void *)&address, xMaxExpectedBlockTime) == pdPASS)
    {
      queue_data_t *queue_data;
      queue_data = (queue_data_t *)address;
      uint16_t cmd_id = (uint16_t)(queue_data->data[0] << 8) + queue_data->data[1];
      // m_judgement.Judgement_Decode(cmd_id, queue_data->data + 2);
      free(queue_data);
    }
    else
    {
      continue;
    }
  }
  /* USER CODE END StartRemoteDecode */
}
/* USER CODE END Application */
