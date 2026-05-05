/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 战队库规范重构版：多轴协同+原生气动 吸取与放置全自动流程
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_dwt.h"
#include "daemon.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "string.h"

// 引入战队核心模块库
#include "DJI_motor.h"
#include "remote.h"
#include "relay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ----------------- 适配大疆库的物理参数 (单位由 Ticks 改为 度 Degree) -----------------
// DJI_motor 库底层会将编码器换算成角度：1圈 = 360.0f 度
#define LIFT_TARGET_30     (20.0f * 360.0f)   // ID1: 第一阶段抬升 20 圈
#define LIFT_TARGET_40     (60.0f * 360.0f)   // ID1: 第二阶段抬升至 60 圈
#define EXTEND_TARGET_OUT  (13.0f * 360.0f)   // ID3: 伸出 13 圈
#define EXTEND_TARGET_IN   (-10.0f * 360.0f)  // ID3: 缩回 -10 圈归零

/* 旋转/俯仰 ID2 参数 */
#define PITCH_45_TARGET    (-26.5f * 360.0f)  // ID2: 45° 对应 -26.5 圈
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

// --- 战队库实例指针 ---
DJIMotor_Instance *motor_lift;     // ID1 抬升
DJIMotor_Instance *motor_pitch;    // ID2 俯仰
DJIMotor_Instance *motor_extend;   // ID3 伸缩
Relay_Instance    *relay_catch;    // 气动继电器
RC_ctrl_t         *rc_data;        // 遥控器数据

// --- 状态机控制变量 ---
uint8_t sys_state = 0;               
uint32_t state_timer = 0;            

// 各轴目标值 (全部变为 float 类型，单位：度)
float target_pos_lift   = 0.0f;
float target_pos_pitch  = 0.0f;
float target_pos_extend = 0.0f;
float current_servo_angle = 135.0f;  

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
void Set_Servo_Angle(float angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 舵机底层映射：输入 0~270，对应脉宽 500~2500。
void Set_Servo_Angle(float angle) {
    if(angle < 0) angle = 0;
    if(angle > 270.0f) angle = 270.0f;
    uint32_t pwm_val = 500 + (uint32_t)((angle / 270.0f) * 2000.0f);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_val);
}

/* USER CODE END 0 */

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
  DWT_Init(168); // F407 主频为 168MHz，提供高精度时间戳给 PID 使用

  // ==================== 1. 初始化气动继电器 ====================
  Relay_Init_Config_s relay_conf = {0};
  relay_conf.gpio1 = GPIOB;
  relay_conf.gpio_pin1 = GPIO_PIN_12;
  relay_catch = RelayInit(&relay_conf);
  RelayOff(relay_catch); // 上电默认放气 (如果你的引脚电平相反，请换成 RelayUp)

  // ==================== 2. 初始化大疆遥控器 ====================
  rc_data = RemoteControlInit(&huart1);

  // ==================== 3. 规范化注册大疆电机 ====================
  Motor_Init_Config_s motor_conf;
  
  // -- 公共配置 --
  memset(&motor_conf, 0, sizeof(Motor_Init_Config_s));
  motor_conf.can_init_config.can_handle = &hcan1; 
  motor_conf.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  motor_conf.controller_setting_init_config.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
  motor_conf.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  motor_conf.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
  motor_conf.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  motor_conf.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;

  // 公共 PID 参数 (位置环+速度环)
  motor_conf.controller_param_init_config.angle_PID.Kp = 0.3f;
  motor_conf.controller_param_init_config.angle_PID.MaxOut = 4000;
  motor_conf.controller_param_init_config.speed_PID.Kp = 5.0f;
  motor_conf.controller_param_init_config.speed_PID.Ki = 0.1f;
  motor_conf.controller_param_init_config.speed_PID.MaxOut = 8000;
  motor_conf.controller_param_init_config.speed_PID.IntegralLimit = 5000;

  // -- 注册 ID1 抬升电机 --
  motor_conf.motor_type = M3508;
  motor_conf.can_init_config.rx_id = 0x201;
  motor_lift = DJIMotorInit(&motor_conf);
  DJIMotorEnable(motor_lift);

  // -- 注册 ID2 俯仰电机 --
  motor_conf.can_init_config.rx_id = 0x202;
  motor_conf.controller_param_init_config.angle_PID.Kp = 0.8f; // 重载电机增加刚性
  motor_pitch = DJIMotorInit(&motor_conf);
  DJIMotorEnable(motor_pitch);

  // -- 注册 ID3 伸缩电机 --
  motor_conf.can_init_config.rx_id = 0x203;
  motor_conf.controller_param_init_config.angle_PID.Kp = 0.3f; // 恢复 0.3
  motor_extend = DJIMotorInit(&motor_conf);
  DJIMotorEnable(motor_extend);


  // 启动 CAN 总线
  HAL_CAN_Start(&hcan1);
  
  // 启动舵机与定时器
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  Set_Servo_Angle(135.0f); // 舵机锁定在 0°

  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

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
  huart1.Init.BaudRate = 100000; 
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN; 
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
  __HAL_RCC_GPIOE_CLK_ENABLE(); 
}

/* USER CODE BEGIN 4 */

/* ================= 1ms 定时器：战队库框架下的状态机 ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {

        DaemonTask(); // 不断更新各个模块的在线状态
        // ID2 始终锁定，仅在剧本中被修改
        target_pos_pitch = 0.0f; 

        switch(sys_state) {
            case 0: // 抬升 20 圈 (3秒)
                state_timer++;
                target_pos_lift = (LIFT_TARGET_30 * (float)state_timer) / 3000.0f;
                if(state_timer >= 3000) { sys_state = 1; state_timer = 0; target_pos_lift = LIFT_TARGET_30; }
                break;
                
            case 1: // 静止 2s
                state_timer++;
                if(state_timer >= 2000) { sys_state = 2; state_timer = 0; }
                break;

            case 2: // 伸出 (2秒)
                state_timer++;
                target_pos_extend = (EXTEND_TARGET_OUT * (float)state_timer) / 2000.0f;
                if(state_timer >= 2000) { sys_state = 3; state_timer = 0; target_pos_extend = EXTEND_TARGET_OUT; }
                break;
                
            case 3: // 静止 2s
                state_timer++;
                if(state_timer >= 2000) { sys_state = 4; state_timer = 0; }
                break;
                
            case 4: // 开始吸气 (调用战队 Relay 库)
                RelayUp(relay_catch); // 继电器动作吸合 (如果是低电平有效且接反，请改用 RelayDown)
                sys_state = 5;
                state_timer = 0;
                break;
                
            case 5: // 静止 2s (稳固吸附)
                state_timer++;
                if(state_timer >= 2000) { sys_state = 6; state_timer = 0; }
                break;
                
            case 6: // 继续抬升至 60 圈 (5秒平滑)
                state_timer++;
                {
                    float lift_time_ms = 5000.0f;
                    float remain_dist = LIFT_TARGET_40 - LIFT_TARGET_30;
                    target_pos_lift = LIFT_TARGET_30 + (remain_dist * ((float)state_timer / lift_time_ms));
                    if(state_timer >= (uint32_t)lift_time_ms) { 
                        sys_state = 7; state_timer = 0; target_pos_lift = LIFT_TARGET_40; 
                    }
                }
                break;
                
            case 7: // 静止 2s
                state_timer++;
                if(state_timer >= 2000) { sys_state = 8; state_timer = 0; }
                break;
                
            case 8: // ID2 缓慢向上运动 45° (3秒)
                state_timer++;
                target_pos_pitch = PITCH_45_TARGET * ((float)state_timer / 3000.0f);
                if(state_timer >= 3000) { sys_state = 9; state_timer = 0; target_pos_pitch = PITCH_45_TARGET; }
                break;
                
            case 9: // 静止 2s
                state_timer++;
                if(state_timer >= 2000) { sys_state = 10; state_timer = 0; }
                break;
                
            case 10: // ID2 缓慢转回 0° (3秒)
                state_timer++;
                target_pos_pitch = PITCH_45_TARGET - (PITCH_45_TARGET * ((float)state_timer / 3000.0f));
                if(state_timer >= 3000) { sys_state = 11; state_timer = 0; target_pos_pitch = 0.0f; }
                break;
                
            case 11: // 静止 2s
                state_timer++;
                if(state_timer >= 2000) { sys_state = 12; state_timer = 0; }
                break;

            case 12: // 放气释放
                RelayOff(relay_catch); 
                sys_state = 13;
                state_timer = 0;
                break;

            case 13: // 下降 & 缩回归零 (4s)
                state_timer++;
                target_pos_lift = LIFT_TARGET_40 - (LIFT_TARGET_40 * (float)state_timer / 4000.0f);
                // 总行程为 23 圈 (13 降到 -10)
                float travel_dist = EXTEND_TARGET_OUT - EXTEND_TARGET_IN;
                target_pos_extend = EXTEND_TARGET_OUT - (travel_dist * (float)state_timer / 4000.0f);
                
                if(state_timer >= 4000) { 
                    sys_state = 14; 
                    state_timer = 0; 
                    target_pos_lift = 0.0f;
                    target_pos_extend = EXTEND_TARGET_IN;
                }
                break;

            case 14: // 结尾停顿 & 调库清除伸缩误差
                state_timer++;
                // 战队库专属：消除电机由于撞击限位带来的内部记忆误差
                DJIMotorReset(motor_extend);
                target_pos_extend = 0.0f;
                
                if(state_timer >= 2000) { sys_state = 0; state_timer = 0; }
                break;
        }

        // 1. 设置舵机 (外部手动实现)
        Set_Servo_Angle(current_servo_angle);

        // 2. 将计算好的期望浮点值下发给大疆库实例
        DJIMotorSetRef(motor_lift, target_pos_lift);
        DJIMotorSetRef(motor_pitch, target_pos_pitch);
        DJIMotorSetRef(motor_extend, target_pos_extend);

        // 3. 一键调用大疆核心处理，内部包含算 PID、组 CAN 封包并发送！
        DJIMotorControl();
    }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}