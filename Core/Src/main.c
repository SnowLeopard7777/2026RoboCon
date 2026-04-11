/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : R2 单电机(ID3)伸缩机构测试 - 17圈极限伸缩与抗拒外力保持
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
// 电机尾部（转子）转17圈对应的绝对 Ticks 数
// 【注意】如果伸出方向是反的，请改成 -(17 * 8192)
#define EXTEND_POS  (17 * 8192) 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
Motor_Measure_t motor[4]; // 数组索引：2对应ID3
PID_t pos_pid[4], spd_pid[4];

int32_t target_pos = 0;      // 目标位置
float target_speed = 0;      // 目标速度
int16_t send_current = 0;    // 发送电流

uint8_t is_init = 0;
int32_t offset_ecd = 0;

uint8_t arm_state = 0;       // 状态机阶段
uint32_t state_timer = 0;    // 计时器 (单位:ms)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);

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

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  // ================= PID 初始化 (仅配置 ID3，数组索引为 2) =================
  pos_pid[2].Kp = 0.15f; 
  pos_pid[2].max_output = 3000; // 最快 3000 RPM 伸出

  spd_pid[2].Kp = 5.0f;  // 速度环比例稍微加大，抗拒外力更强
  spd_pid[2].Ki = 0.1f; 
  spd_pid[2].max_output = 8000; // 允许输出大电流抵抗外力

  CAN_Filter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  // 启动控制心跳
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
// ================= CAN 接收：上电归零与多圈解算 =================
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    // 【修改点】：只过滤接收 ID3 (0x203) 的数据
    if (rx_header.StdId == 0x203) {
        uint16_t current_ecd = (rx_data[0] << 8) | rx_data[1];
        
        // 上电第一次记录初始位置
        if (!is_init) {
            offset_ecd = current_ecd;
            is_init = 1;
            return;
        }

        motor[2].last_ecd = motor[2].ecd;
        motor[2].ecd = current_ecd;
        motor[2].speed_rpm = (rx_data[2] << 8) | rx_data[3];
        
        int16_t diff = motor[2].ecd - motor[2].last_ecd;
        if (diff < -4096) motor[2].total_round++;
        else if (diff > 4096) motor[2].total_round--;
        
        motor[2].total_ecd = motor[2].total_round * 8192 + motor[2].ecd - offset_ecd;
    }
}

// ================= 1ms 定时器：状态机与串级 PID =================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        
        // --- 核心时序状态机 (加入线性轨迹规划) ---
        switch(arm_state) {
            case 0: // 阶段1：3秒内匀速伸出到最长处
                state_timer++;
                // 线性插值：当前目标位置 = (总距离 * 当前时间) / 总时间
                target_pos = (EXTEND_POS * state_timer) / 3000; 
                
                if(state_timer >= 3000) { // 3000ms = 3s 走完
                    target_pos = EXTEND_POS; // 确保最后对齐
                    arm_state = 1; 
                    state_timer = 0; 
                }
                break;
                
            case 1: // 阶段2：到达最长处，保持静止并抵抗外力 5 秒
                target_pos = EXTEND_POS; 
                state_timer++;
                if(state_timer >= 5000) { // 5000ms = 5s
                    arm_state = 2; 
                    state_timer = 0; 
                }
                break;
                
            case 2: // 阶段3：3秒内匀速缩回到最短处
                state_timer++;
                // 线性插值：当前目标位置 = 总距离 - (总距离 * 当前时间) / 总时间
                target_pos = EXTEND_POS - (EXTEND_POS * state_timer) / 3000;
                
                if(state_timer >= 3000) { 
                    target_pos = 0; // 确保最后归零
                    arm_state = 3; 
                    state_timer = 0; 
                }
                break;
                
            case 3: // 阶段4：在最短处停顿 2 秒后循环
                target_pos = 0;
                state_timer++;
                if(state_timer >= 2000) { 
                    arm_state = 0; 
                    state_timer = 0; 
                }
                break;
        }

        // --- 串级 PID 计算 (使用索引 2) ---
        target_speed = PID_Calc(&pos_pid[2], (float)target_pos, (float)motor[2].total_ecd);
        send_current = (int16_t)PID_Calc(&spd_pid[2], target_speed, (float)motor[2].speed_rpm);

        // --- CAN 打包与发送 ---
        CAN_TxHeaderTypeDef tx_header = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8};
        uint8_t tx_data[8] = {0};
        uint32_t tx_mailbox;
        
        // 封包 ID3，对应 data[4] 和 data[5]
        tx_data[4] = send_current >> 8; 
        tx_data[5] = send_current & 0xFF;
        
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