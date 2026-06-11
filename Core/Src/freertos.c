/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
#include "dji_motor.h"
#include "remote.h"
#include "elevate_control.h"

__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_dji_motor_task_tick = 0U;

/* Definitions for Elevate_Task */
osThreadId_t Elevate_TaskHandle;
const osThreadAttr_t Elevate_Task_attributes = {
  .name = "Elevate_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for DJIMotor_Task */
osThreadId_t DJIMotor_TaskHandle;
const osThreadAttr_t DJIMotor_Task_attributes = {
  .name = "DJIMotor_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for Remot_Task */
osThreadId_t Remot_TaskHandle;
const osThreadAttr_t Remot_Task_attributes = {
  .name = "Remot_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static void ElevateTaskEntry(void *argument);
static void DJIMotorTaskEntry(void *argument);
static void StartRemote(void *argument);

void MX_FREERTOS_Init(void)
{
  Elevate_TaskHandle = osThreadNew(ElevateTaskEntry, NULL, &Elevate_Task_attributes);
  DJIMotor_TaskHandle = osThreadNew(DJIMotorTaskEntry, NULL, &DJIMotor_Task_attributes);
  Remot_TaskHandle = osThreadNew(StartRemote, NULL, &Remot_Task_attributes);
}

static void ElevateTaskEntry(void *argument)
{
  (void)argument;

  for (;;)
  {
    ElevateControl_Task();
    osDelay(1);
  }
}

static void DJIMotorTaskEntry(void *argument)
{
  (void)argument;

  for (;;)
  {
    g_dji_motor_task_tick++;
    DJIMotorControl();
    osDelay(1);
  }
}

static void StartRemote(void *argument)
{
  (void)argument;

  for (;;)
  {
    RemoteControlTask();
  }
}
