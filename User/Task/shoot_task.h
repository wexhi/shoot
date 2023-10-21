#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "gpio.h"

void Shoot_task(void const *pvParameters);
void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed);
void shoot_current_give(void);
#endif
