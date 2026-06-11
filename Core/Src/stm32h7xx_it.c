/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32h7xx_it.h"
#include "remote.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim7;

void NMI_Handler(void)
{
  while (1) {}
}

void HardFault_Handler(void)
{
  while (1) {}
}

void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void DebugMon_Handler(void)
{
}

void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
}

void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN1_IT1_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}

void TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim7);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  Remote_RxCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Remote_ErrorCallback(huart);
}
