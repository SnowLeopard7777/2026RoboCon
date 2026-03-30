#include "main.h"
#include "math.h"

CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;

typedef struct{    //Define Motor_Data
    int16_t speed_rpm;
    int16_t real_current;
    unit16_t angle;
    unit16_t temperature;
}motor_Measure_t;
motor_Measure_t M3508;

typedef struct{
    float Kp,Ki,Kd;
    float error,last_error;
    float integral;
    float output;
    float max_out;
    float max_integral;
}PID_Controler_t;

PID_Controler_t motor_pid{
    .Kp = 5.0f
    .ki = 0.1f
    .kd = 0.0f
    .max_out = 10000.0f
    .max_integral = 5000.0f
};

unit32_t time_tick = 0;
float target_speed = 0.0f;

