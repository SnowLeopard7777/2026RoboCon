#include "lift.h"
#include "stdlib.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "bsp_pwm.h"

ServoMotor*servo;

//舵机
ServoMotor* ServoInit( TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch) {
    ServoMotor* servo = (ServoMotor*)malloc(sizeof(ServoMotor));

    // PWM初始化
    PWM_Init_Config_s pwm_conf = {
        .htim = pwm_tim,
        .channel = pwm_ch,
		.period = 50,
    };
   servo->pwm = PWMRegister(&pwm_conf);

    return servo;
}


// 舵机控制
void ServoControl(ServoMotor* servo,float position) {


    // 计算周期并配置PWM
    if(position > 0) {
			  PWMStart(servo->pwm);
			  __HAL_TIM_SET_COMPARE(servo->pwm->htim,servo->pwm->channel,position);

    } else {
        PWMStop(servo->pwm);
    }


}


void servo_init(){
	servo = ServoInit(&htim1, TIM_CHANNEL_4);
}


void servo_control(float position){
	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1500-(position*2000.0f)/270.0f);
}
