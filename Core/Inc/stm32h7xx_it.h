/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __STM32H7xx_IT_H
#define __STM32H7xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void DMA1_Stream0_IRQHandler(void);
void FDCAN1_IT0_IRQHandler(void);
void FDCAN1_IT1_IRQHandler(void);
void TIM7_IRQHandler(void);
void USART6_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
