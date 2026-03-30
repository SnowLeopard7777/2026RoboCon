/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : M3508 双电机串级 PID 与时序控制实战 (带减速比与上电归零)
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
    int32_t total_ecd;     // 核心位置数据 (带符号的连续累计值)
} Motor_Measure_t;

typedef struct {
    float Kp, Ki, Kd;
    float error, last_error;
    float integral;
    float max_integral;
    float output;
    float max_output;
} PID_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// M3508 减速比 19:1，输出轴转 1 圈对应的内部 tick 数
#define ONE_TURN (19 * 8192) 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
Motor_Measure_t motor[2];  // 0:升降(ID1), 1:旋转(ID2)

PID_t pos_pid[2];          
PID_t spd_pid[2];          

int32_t target_pos[2] = {0, 0};     
float target_speed[2] = {0, 0};     
int16_t send_current[2] = {0, 0};   

// 上电归零补丁变量
uint8_t is_init[2] = {0, 0};
int32_t offset_ecd[2] = {0, 0};

// 状态机控制变量
uint8_t arm_state = 0;       
uint32_t state_timer = 0;    
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
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

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // ---------------- 参数初始化区 ----------------
  // ID1 升降电机：空载测试相对温柔的参数
  pos_pid[0].Kp = 0.1f;   pos_pid[0].Ki = 0.0f; pos_pid[0].Kd = 0.0f;
  pos_pid[0].max_output = 3000; // 最大速度 3000 RPM
  spd_pid[0].Kp = 3.0f;   spd_pid[0].Ki = 0.05f; spd_pid[0].Kd = 0.0f;
  spd_pid[0].max_integral = 2000; spd_pid[0].max_output = 5000; // 最大电流 5000

  // ID2 旋转电机
  pos_pid[1].Kp = 0.1f;   pos_pid[1].Ki = 0.0f; pos_pid[1].Kd = 0.0f;
  pos_pid[1].max_output = 3000; 
  spd_pid[1].Kp = 3.0f;   spd_pid[1].Ki = 0.05f; spd_pid[1].Kd = 0.0f;
  spd_pid[1].max_integral = 2000; spd_pid[1].max_output = 5000;

  // 启动 CAN 和定时器
  CAN_Filter_Init(&hcan1); 
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // VOFA+ 发送代码 (可观察 ID1 的位置追踪)
    float vofa_buf[2];
    vofa_buf[0] = (float)target_pos[0];      // 目标位置
    vofa_buf[1] = (float)motor[0].total_ecd; // 实际位置

    HAL_UART_Transmit(&huart1, (uint8_t*)vofa_buf, sizeof(vofa_buf), 10);
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    HAL_UART_Transmit(&huart1, tail, 4, 10);

    HAL_Delay(10); 
  }
  /* USER CODE END 3 */
}9

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
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
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ================= CAN 接收：带上电归零的多圈解算 =================
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x202) {
        uint8_t i = rx_header.StdId - 0x201; 
        uint16_t current_ecd = (rx_data[0] << 8) | rx_data[1];
        
        // 上电第一次收到数据，记录初始偏置
        if (is_init[i] == 0) {
            motor[i].ecd = current_ecd;
            motor[i].last_ecd = current_ecd;
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
        
        // 计算绝对连续位置 (减去上电偏置，保证起步位置为 0)
        motor[i].total_ecd = motor[i].total_round * 8192 + motor[i].ecd - offset_ecd[i];
    }
}

// ================= 1ms 定时器：状态机与串级 PID =================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        // 容差设定：约输出轴 4.6 度
        uint8_t is_reached = (abs(motor[0].total_ecd - target_pos[0]) < 2000) && 
                             (abs(motor[1].total_ecd - target_pos[1]) < 2000);

        switch(arm_state) {
            case 0: // 阶段一：ID1 抬升 30 圈 (逆时针负方向)
                target_pos[0] = -30 * ONE_TURN; 
                target_pos[1] = 0;          
                if(is_reached) {
                    state_timer++;
                    if(state_timer >= 2000) { arm_state = 1; state_timer = 0; }
                } else { state_timer = 0; }
                break;
                
            case 1: // 阶段二：ID1 保持，ID2 转 20 圈 (逆时针负方向)
                target_pos[0] = -30 * ONE_TURN;
                target_pos[1] = -20 * ONE_TURN; 
                if(is_reached) {
                    state_timer++;
                    if(state_timer >= 2000) { arm_state = 2; state_timer = 0; }
                } else { state_timer = 0; }
                break;
                
            case 2: // 阶段三：反向操作，ID2 回 0
                target_pos[0] = -30 * ONE_TURN;
                target_pos[1] = 0;
                if(is_reached) {
                    state_timer++;
                    if(state_timer >= 2000) { arm_state = 3; state_timer = 0; }
                } else { state_timer = 0; }
                break;
                
            case 3: // 阶段四：ID1 回 0
                target_pos[0] = 0;
                target_pos[1] = 0;
                if(is_reached) {
                    state_timer++;
                    if(state_timer >= 2000) { arm_state = 0; state_timer = 0; } 
                } else { state_timer = 0; }
                break;
        }

        for(int i = 0; i < 2; i++) {
            target_speed[i] = PID_Calc(&pos_pid[i], (float)target_pos[i], (float)motor[i].total_ecd);
            send_current[i] = (int16_t)PID_Calc(&spd_pid[i], target_speed[i], (float)motor[i].speed_rpm);
        }

        CAN_TxHeaderTypeDef tx_header;
        uint8_t tx_data[8] = {0};
        uint32_t tx_mailbox;

        tx_header.StdId = 0x200;  
        tx_header.ExtId = 0;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8;

        tx_data[0] = send_current[0] >> 8;   
        tx_data[1] = send_current[0] & 0xFF; 
        tx_data[2] = send_current[1] >> 8;   
        tx_data[3] = send_current[1] & 0xFF; 

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
