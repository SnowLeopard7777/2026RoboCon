#ifndef __INS_TASK_H__
#define __INS_TASK_H__

/* 这是一个桥接文件，用于骗过 robot_def.h 的套娃依赖 */
/* 随便给它捏造一个姿态结构体，反正在咱们这个项目里用不到 */
typedef struct {
    float yaw;
    float pitch;
    float roll;
} attitude_t;

#endif
