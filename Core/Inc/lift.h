#ifndef __ARM_H
#define __ARM_H

#include "main.h"
#include "bsp_pwm.h"

typedef struct {

    PWMInstance *pwm;

} ServoMotor;

// 全局舵机实例
extern ServoMotor *servo;


// 舵机基础函数
ServoMotor* ServoInit(TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch);
void ServoControl(ServoMotor* servo, float position);

// 舵机初始化函数
void servo_init(void);

// 舵机控制函数
void servo_control(float position);


// 电机初始化和控制函数
void Arm_motor_Init(void);
void Arm_motor1_Task(float angle);
void Arm_motor2_Task(float angle);
void Arm_motor3_Task(float angle);




#endif
