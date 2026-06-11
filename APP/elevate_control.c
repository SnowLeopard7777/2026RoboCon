#include "elevate_control.h"

#include "bsp_fdcan.h"
#include "dji_motor.h"

/* ====== Elevate test parameters: adjust here ====== */
#define ELEVATE_TEST_UP_TIME_MS         3000U
#define ELEVATE_TEST_DOWN_TIME_MS       3000U
#define ELEVATE_MOTOR_ID1_UP_SPEED_RPM  -1500.0f
#define ELEVATE_MOTOR_ID1_DOWN_SPEED_RPM 1500.0f
#define ELEVATE_MOTOR_ID3_UP_SPEED_RPM   1500.0f
#define ELEVATE_MOTOR_ID3_DOWN_SPEED_RPM -1500.0f
/* ====== Elevate test parameters: adjust here ====== */

#define ELEVATE_MOTOR_STOP_SPEED_RPM        0.0f

static DJIMotor_Instance *g_left_elevate_motor = 0;
static DJIMotor_Instance *g_right_elevate_motor = 0;
static uint32_t g_elevate_test_start_tick = 0U;

__attribute__((section(".ram_d2_nocache"))) volatile float g_elevate_dbg_motor1_ref_rpm = 0.0f;
__attribute__((section(".ram_d2_nocache"))) volatile float g_elevate_dbg_motor3_ref_rpm = 0.0f;
__attribute__((section(".ram_d2_nocache"))) volatile int16_t g_elevate_dbg_motor1_speed_rpm = 0;
__attribute__((section(".ram_d2_nocache"))) volatile int16_t g_elevate_dbg_motor3_speed_rpm = 0;
__attribute__((section(".ram_d2_nocache"))) volatile int32_t g_elevate_dbg_motor1_total_ecd = 0;
__attribute__((section(".ram_d2_nocache"))) volatile int32_t g_elevate_dbg_motor3_total_ecd = 0;
__attribute__((section(".ram_d2_nocache"))) volatile float g_elevate_dbg_motor1_total_angle = 0.0f;
__attribute__((section(".ram_d2_nocache"))) volatile float g_elevate_dbg_motor3_total_angle = 0.0f;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_elevate_dbg_motor_init_ok_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_elevate_dbg_motor_init_fail_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint8_t g_elevate_dbg_motor_last_init_stage = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_elevate_dbg_task_tick = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_elevate_dbg_can_last_tx_id = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_elevate_dbg_can_last_rx_id = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_elevate_dbg_can_rx_match_count = 0U;
__attribute__((section(".ram_d2_nocache"))) volatile uint32_t g_elevate_dbg_can_rx_unmatched_count = 0U;

static void ElevateMotor_InitInstance(DJIMotor_Instance **motor, uint32_t tx_id)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan1,
            .tx_id = tx_id,
            .rx_id = 0,
            .use_canfd = 0,
            .can_module_callback = 0,
            .id = 0,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 0.0f,
                .DeadBand = 0.0f,
                .Improve = PID_IMPROVE_NONE,
            },
            .speed_PID = {
                .Kp = 8.0f,
                .Ki = 0.12f,
                .Kd = 0.0f,
                .MaxOut = 16000.0f,
                .IntegralLimit = 8000.0f,
                .DeadBand = 10.0f,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
            },
            .current_PID = {
                .Kp = 0.9f,
                .Ki = 0.05f,
                .Kd = 0.0f,
                .MaxOut = 16384.0f,
                .IntegralLimit = 12000.0f,
                .DeadBand = 5.0f,
                .Improve = PID_Integral_Limit,
            },
        },
        .controller_setting_init_config = {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .output_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .angle_mode = MOTOR_ANGLE_MODE_TOTAL,
            .feedforward_flag = FEEDFORWARD_NONE,
            .angle_ramp_flag = MOTOR_RAMP_DISABLE,
            .speed_ramp_flag = MOTOR_RAMP_DISABLE,
        },
        .motor_type = M3508,
    };

    *motor = DJIMotorInit(&config);
    if (*motor != 0) {
        DJIMotorOuterLoop(*motor, SPEED_LOOP);
        DJIMotorEnable(*motor);
        DJIMotorSetRef(*motor, ELEVATE_MOTOR_STOP_SPEED_RPM);
    }
}

static float ElevateControl_GetMotor1SpeedRef(void)
{
    const uint32_t cycle_time_ms = ELEVATE_TEST_UP_TIME_MS + ELEVATE_TEST_DOWN_TIME_MS;
    const uint32_t elapsed_ms = (HAL_GetTick() - g_elevate_test_start_tick) % cycle_time_ms;

    if (elapsed_ms < ELEVATE_TEST_UP_TIME_MS) {
        return ELEVATE_MOTOR_ID1_UP_SPEED_RPM;
    }

    return ELEVATE_MOTOR_ID1_DOWN_SPEED_RPM;
}

static float ElevateControl_GetMotor3SpeedRef(void)
{
    const uint32_t cycle_time_ms = ELEVATE_TEST_UP_TIME_MS + ELEVATE_TEST_DOWN_TIME_MS;
    const uint32_t elapsed_ms = (HAL_GetTick() - g_elevate_test_start_tick) % cycle_time_ms;

    if (elapsed_ms < ELEVATE_TEST_UP_TIME_MS) {
        return ELEVATE_MOTOR_ID3_UP_SPEED_RPM;
    }

    return ELEVATE_MOTOR_ID3_DOWN_SPEED_RPM;
}

void ElevateControl_Init(void)
{
    ElevateMotor_InitInstance(&g_left_elevate_motor, 1U);
    ElevateMotor_InitInstance(&g_right_elevate_motor, 3U);
    g_elevate_test_start_tick = HAL_GetTick();
    g_elevate_dbg_motor_init_ok_count = g_dji_motor_debug.init_ok_count;
    g_elevate_dbg_motor_init_fail_count = g_dji_motor_debug.init_fail_count;
    g_elevate_dbg_motor_last_init_stage = g_dji_motor_debug.last_init_stage;
}

void ElevateControl_Task(void)
{
    const float motor1_speed_ref = ElevateControl_GetMotor1SpeedRef();
    const float motor3_speed_ref = ElevateControl_GetMotor3SpeedRef();

    g_elevate_dbg_task_tick++;
    g_elevate_dbg_motor1_ref_rpm = motor1_speed_ref;
    g_elevate_dbg_motor3_ref_rpm = motor3_speed_ref;

    if (g_left_elevate_motor != 0) {
        DJIMotorEnable(g_left_elevate_motor);
        DJIMotorSetRef(g_left_elevate_motor, motor1_speed_ref);
        g_elevate_dbg_motor1_speed_rpm = g_left_elevate_motor->measure.speed_rpm;
        g_elevate_dbg_motor1_total_ecd = g_left_elevate_motor->measure.total_ecd;
        g_elevate_dbg_motor1_total_angle = g_left_elevate_motor->measure.total_angle;
    }

    if (g_right_elevate_motor != 0) {
        DJIMotorEnable(g_right_elevate_motor);
        DJIMotorSetRef(g_right_elevate_motor, motor3_speed_ref);
        g_elevate_dbg_motor3_speed_rpm = g_right_elevate_motor->measure.speed_rpm;
        g_elevate_dbg_motor3_total_ecd = g_right_elevate_motor->measure.total_ecd;
        g_elevate_dbg_motor3_total_angle = g_right_elevate_motor->measure.total_angle;
    }

    g_elevate_dbg_can_last_tx_id = g_fdcan1_debug.last_tx_id;
    g_elevate_dbg_can_last_rx_id = g_fdcan1_debug.last_rx_id;
    g_elevate_dbg_can_rx_match_count = g_fdcan1_debug.rx_match_count;
    g_elevate_dbg_can_rx_unmatched_count = g_fdcan1_debug.rx_unmatched_count;
    g_elevate_dbg_motor_init_ok_count = g_dji_motor_debug.init_ok_count;
    g_elevate_dbg_motor_init_fail_count = g_dji_motor_debug.init_fail_count;
    g_elevate_dbg_motor_last_init_stage = g_dji_motor_debug.last_init_stage;
}
