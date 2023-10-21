#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"

typedef enum
{
    CHAS_LF,
    CHAS_RF,
    CHAS_RB,
    CHAS_LB,
} chassis_motor_cnt_t;

extern int16_t Drifting_yaw;
extern uint16_t Down_ins_yaw;

void Shoot_task(void const *pvParameters);
void chassis_motol_speed_calculate(void);
void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed);
void chassis_current_give(void);
#endif
