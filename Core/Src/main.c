/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : R2 自动化机械臂测试：旋转 + 伸缩 + 舵机 (完整无错版)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t speed_rpm;
    uint16_t ecd;          
    uint16_t last_ecd;     
    int32_t total_round;   
    int32_t total_ecd;     
} Motor_Measure_t;

typedef struct {
    float Kp, Ki, Kd;
    float error, last_error;
    float integral, max_integral;
    float output, max_output;
} PID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 旋转：(90/360) * 1.5 * 19 * 8192
#define POS_ROTATE_90  (58368)
// 伸缩：(20cm / (2*PI*2cm)) * 19 * 8192 
#define POS_EXTEND_20  (247713)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
Motor_Measure_t motor[4]; // 数组索引：1对应ID2(旋转)，2对应ID3(伸缩)
PID_t pos_pid[4], spd_pid[4];
int32_t target_pos[4] = {0};
float target_speed[4] = {0};
int16_t send_current[4] = {0};

uint8_t is_init[4] = {0};
int32_t offset_ecd[4] = {0};
uint8_t arm_state = 0;       
uint32_t state_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
float PID_Calc(PID_t *pid, float target, float measure);
void CAN_Filter_Init(CAN_HandleTypeDef *hcan);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float PID_Calc(PID_t *pid, float target, float measure) {
    pid->error = target - measure;
    pid->integral += pid->error;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;
    if(pid->output > pid->max_output) pid->output = pid->max_output;
    else if(pid->output < -pid->max_output) pid->output = -pid->max_output;
    return pid->output;
}

void CAN_Filter_Init(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef can_filter;
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &can_filter);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  // PID 初始化
  for(int i=1; i<=2; i++) { // 为 ID2 和 ID3 赋予参数
    pos_pid[i].Kp = 0.15f; pos_pid[i].max_output = 3000;
    spd_pid[i].Kp = 4.0f;  spd_pid[i].Ki = 0.1f; spd_pid[i].max_output = 8000;
  }

  CAN_Filter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // 开启舵机 PWM

  // 初始舵机位置：中位 (1.5ms)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
// ================= CAN 接收：带上电归零的多圈解算 =================
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.StdId >= 0x202 && rx_header.StdId <= 0x203) {
        uint8_t i = rx_header.StdId - 0x201; // ID2->1, ID3->2
        uint16_t current_ecd = (rx_data[0] << 8) | rx_data[1];
        
        if (!is_init[i]) {
            offset_ecd[i] = current_ecd;
            is_init[i] = 1;
            return;
        }

        motor[i].last_ecd = motor[i].ecd;
        motor[i].ecd = current_ecd;
        motor[i].speed_rpm = (rx_data[2] << 8) | rx_data[3];
        
        int16_t diff = motor[i].ecd - motor[i].last_ecd;
        if (diff < -4096) motor[i].total_round++;
        else if (diff > 4096) motor[i].total_round--;
        
        motor[i].total_ecd = motor[i].total_round * 8192 + motor[i].ecd - offset_ecd[i];
    }
}

// ================= 1ms 定时器：状态机与串级 PID =================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        // 运动完成判定 (容差1000 tick)
        uint8_t rotate_done = abs(motor[1].total_ecd - target_pos[1]) < 1000;
        uint8_t extend_done = abs(motor[2].total_ecd - target_pos[2]) < 1000;

        switch(arm_state) {
            case 0: // 阶段1：旋转 90 度 (逆时针)
                target_pos[1] = -POS_ROTATE_90;
                target_pos[2] = 0;
                if(rotate_done) { arm_state = 1; state_timer = 0; }
                break;
                
            case 1: // 阶段2：伸出 20cm (逆时针)
                target_pos[1] = -POS_ROTATE_90;
                target_pos[2] = -POS_EXTEND_20;
                if(extend_done) { 
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500); // 舵机中位
                    arm_state = 2; state_timer = 0; 
                }
                break;
                
            case 2: // 阶段3：静止 4s
                state_timer++;
                if(state_timer >= 4000) { arm_state = 3; state_timer = 0; }
                break;
                
            case 3: // 阶段4：原路返回 - 缩回
                target_pos[1] = -POS_ROTATE_90;
                target_pos[2] = 0;
                if(extend_done) { arm_state = 4; state_timer = 0; }
                break;
                
            case 4: // 阶段5：原路返回 - 旋转复位
                target_pos[1] = 0;
                target_pos[2] = 0;
                if(rotate_done) { arm_state = 0; state_timer = 0; } // 循环
                break;
        }

        // PID 计算与 CAN 发送
        for(int i = 1; i <= 2; i++) {
            target_speed[i] = PID_Calc(&pos_pid[i], (float)target_pos[i], (float)motor[i].total_ecd);
            send_current[i] = (int16_t)PID_Calc(&spd_pid[i], target_speed[i], (float)motor[i].speed_rpm);
        }

        CAN_TxHeaderTypeDef tx_header = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8};
        uint8_t tx_data[8] = {0};
        uint32_t tx_mailbox;
        
        // 封包：ID2对应data[2][3], ID3对应data[4][5]
        tx_data[2] = send_current[1] >> 8; 
        tx_data[3] = send_current[1] & 0xFF;
        tx_data[4] = send_current[2] >> 8; 
        tx_data[5] = send_current[2] & 0xFF;
        
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    }
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif /* USE_FULL_ASSERT */