/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_dwt.h"
#include "bsp_fdcan.h"
/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);

__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_main_loop_tick = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_main_hal_tick = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_main_last_loop_period_ms = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_main_mode_bypass_freertos = 1U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_started = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_phase = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile int16_t g_raw_can_motor1_current = 0;
__attribute__((section(".ram_d2_nocache"))) volatile int16_t g_raw_can_motor3_current = 0;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_tx_ok_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_tx_fail_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_last_hal_status = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_last_error_code = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_last_rx_id = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_rx_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_rx_unmatched_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_tx_free_level = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_protocol_last_error = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_protocol_activity = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_protocol_error_passive = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_protocol_warning = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_protocol_bus_off = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_tx_error_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_rx_error_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_raw_can_error_logging = 0U;

/* ====== Raw C620 CAN test parameters: adjust here ====== */
#define RAW_CAN_TEST_UP_TIME_MS             2000U
#define RAW_CAN_TEST_DOWN_TIME_MS           5000U
#define RAW_CAN_TEST_ID1_UP_CURRENT        -8100 
#define RAW_CAN_TEST_ID1_DOWN_CURRENT      -2000
#define RAW_CAN_TEST_ID3_UP_CURRENT         8100
#define RAW_CAN_TEST_ID3_DOWN_CURRENT       2000
#define RAW_CAN_TEST_SEND_PERIOD_MS          10U
/* ====== Raw C620 CAN test parameters: adjust here ====== */

static void RawC620_UpdateCanDebug(void)
{
  FDCAN_ProtocolStatusTypeDef protocol_status = {0};
  FDCAN_ErrorCountersTypeDef error_counters = {0};

  g_raw_can_tx_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
  g_raw_can_last_error_code = hfdcan1.ErrorCode;

  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK) {
    g_raw_can_protocol_last_error = protocol_status.LastErrorCode;
    g_raw_can_protocol_activity = protocol_status.Activity;
    g_raw_can_protocol_error_passive = protocol_status.ErrorPassive;
    g_raw_can_protocol_warning = protocol_status.Warning;
    g_raw_can_protocol_bus_off = protocol_status.BusOff;
  }

  if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
    g_raw_can_tx_error_count = error_counters.TxErrorCnt;
    g_raw_can_rx_error_count = error_counters.RxErrorCnt;
    g_raw_can_error_logging = error_counters.ErrorLogging;
  }
}

static void RawC620_StartCan(void)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    g_raw_can_last_hal_status = 1U;
    g_raw_can_last_error_code = hfdcan1.ErrorCode;
    return;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    g_raw_can_last_hal_status = 2U;
    g_raw_can_last_error_code = hfdcan1.ErrorCode;
    return;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    g_raw_can_last_hal_status = 3U;
    g_raw_can_last_error_code = hfdcan1.ErrorCode;
    return;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    g_raw_can_last_hal_status = 4U;
    g_raw_can_last_error_code = hfdcan1.ErrorCode;
    return;
  }

  g_raw_can_started = 1U;
  g_raw_can_last_hal_status = 0U;
  g_raw_can_last_error_code = 0U;
}

static void RawC620_SendCurrent(int16_t id1_current, int16_t id3_current)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  HAL_StatusTypeDef status;

  RawC620_UpdateCanDebug();

  if (g_raw_can_tx_free_level == 0U) {
    g_raw_can_tx_fail_count++;
    g_raw_can_last_hal_status = 0xF0U;
    RawC620_UpdateCanDebug();
    return;
  }

  tx_header.Identifier = 0x200;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  tx_data[0] = (uint8_t)(id1_current >> 8);
  tx_data[1] = (uint8_t)id1_current;
  tx_data[4] = (uint8_t)(id3_current >> 8);
  tx_data[5] = (uint8_t)id3_current;

  status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
  g_raw_can_last_hal_status = (uint32_t)status;
  g_raw_can_last_error_code = hfdcan1.ErrorCode;
  if (status == HAL_OK) {
    g_raw_can_tx_ok_count++;
  } else {
    g_raw_can_tx_fail_count++;
  }
  RawC620_UpdateCanDebug();
}

int main(void)
{
  uint32_t last_loop_tick = 0U;
  uint32_t last_send_tick = 0U;

  MPU_Config();

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USART6_UART_Init();

  DWT_Init(400);

  RawC620_StartCan();

  while (1)
  {
    const uint32_t current_tick = HAL_GetTick();
    const uint32_t cycle_time_ms = RAW_CAN_TEST_UP_TIME_MS + RAW_CAN_TEST_DOWN_TIME_MS;
    const uint32_t elapsed_ms = current_tick % cycle_time_ms;

    g_main_loop_tick++;
    g_main_hal_tick = current_tick;
    g_main_last_loop_period_ms = current_tick - last_loop_tick;
    last_loop_tick = current_tick;

    if (elapsed_ms < RAW_CAN_TEST_UP_TIME_MS) {
      g_raw_can_phase = 1U;
      g_raw_can_motor1_current = RAW_CAN_TEST_ID1_UP_CURRENT;
      g_raw_can_motor3_current = RAW_CAN_TEST_ID3_UP_CURRENT;
    } else {
      g_raw_can_phase = 2U;
      g_raw_can_motor1_current = RAW_CAN_TEST_ID1_DOWN_CURRENT;
      g_raw_can_motor3_current = RAW_CAN_TEST_ID3_DOWN_CURRENT;

    }

    RawC620_UpdateCanDebug();

    if (g_raw_can_started != 0U &&
        (uint32_t)(current_tick - last_send_tick) >= RAW_CAN_TEST_SEND_PERIOD_MS) {
      last_send_tick = current_tick;
      RawC620_SendCurrent(g_raw_can_motor1_current, g_raw_can_motor3_current);
    }

    g_raw_can_last_rx_id = g_fdcan1_debug.last_rx_id;
    g_raw_can_rx_count = g_fdcan1_debug.rx_match_count;
    g_raw_can_rx_unmatched_count = g_fdcan1_debug.rx_unmatched_count;

    HAL_Delay(1);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                              | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
