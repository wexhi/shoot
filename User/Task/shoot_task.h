#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "gpio.h"

#define PI 3.1415926f // pi
typedef struct
{
    /* data */
    motor_info_t motor_info[3];            // 电机信息结构体
    fp32 speed_motor_pid[3];               // 速度环pid
    fp32 angle_pid[3];                      // 角度环pid
    volatile int16_t fric_speed_target[2]; // 摩擦轮电机目标值
    volatile int16_t shoot_speed_target;   // 拨盘电机目标值
    int16_t angle_target;

    pid_struct_t shoot_motor_pid; // 拨盘电机pid结构体
    pid_struct_t shoot_angle_pid; // 拨盘角度pid结构体
    pid_struct_t friction_pid[2]; // 摩擦轮电机pid结构体
} shoot_task_t;

void shoot_task_init(void);
void Shoot_task(void const *pvParameters);
void shoot_current_give(void);
#endif
