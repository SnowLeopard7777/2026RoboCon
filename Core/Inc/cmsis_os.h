#ifndef __CMSIS_OS_H__
#define __CMSIS_OS_H__
#include "main.h"
#define osDelay HAL_Delay  // 把 RTOS 的延时巧妙替换成裸机延时
#endif
