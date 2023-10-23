
#include "shoot_task.h"
#include "cmsis_os.h"
#include "drv_can.h"
#include <stdlib.h>

extern RC_ctrl_t rc_ctrl;

uint8_t shoot_flag = 0;

shoot_task_t shoot_task;

// 发射机构初始化
void shoot_task_init(void)
{
    shoot_task.speed_motor_pid[0] = 8;   // 速度环pid->kp
    shoot_task.speed_motor_pid[1] = 0.9; // 速度环pid->ki
    shoot_task.speed_motor_pid[2] = 0;   // 速度环pid->kd
    shoot_task.angle_pid[0] = 30;        // 角度环pid->kp
    shoot_task.angle_pid[1] = 0.5;       // 角度环pid->ki
    shoot_task.angle_pid[2] = 10;        // 角度环pid->kd
    shoot_task.angle_target = 0;         // 角度环目标值
    shoot_task.shoot_speed_target = 0;   // 拨盘速度环目标值
    shoot_task.fric_speed_target[0] = 0; // 摩擦轮电机目标值
    shoot_task.fric_speed_target[1] = 0; // 摩擦轮电机目标值

    pid_init(&shoot_task.shoot_angle_pid, shoot_task.angle_pid, 2.0f * PI, 2.0f * PI); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
    pid_init(&shoot_task.shoot_motor_pid, shoot_task.speed_motor_pid, 1500, 1500);     // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
    pid_init(&shoot_task.friction_pid[0], shoot_task.speed_motor_pid, 6000, 6000);     // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
    pid_init(&shoot_task.friction_pid[1], shoot_task.speed_motor_pid, 6000, 6000);     // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384

    // motor_angle_judge(2);
}

// 角度判断
void motor_angle_judge(uint8_t motor_index)
{
    shoot_task.motor_info[motor_index].angle_error = shoot_task.motor_info[motor_index].rotor_angle -
                                                     shoot_task.motor_info[motor_index].angle_old_err;

    if (shoot_task.motor_info->rotor_speed > 0)
    {
        if (abs(shoot_task.motor_info[motor_index].loop_count) < 36) // 没有转满一圈
        {
            if (abs(shoot_task.motor_info[motor_index].angle_error) > 1000) // 转了一圈
            {
                shoot_task.motor_info[motor_index].loop_count++;
            }
        }
        else
        {
            shoot_task.motor_info[motor_index].loop_count = 0;
        }
    }
    else if (shoot_task.motor_info->rotor_speed < 0)
    {
        if (abs(shoot_task.motor_info[motor_index].loop_count) < 36)
        {
            if (abs(shoot_task.motor_info[motor_index].angle_error) > 1000)
            {
                shoot_task.motor_info[motor_index].loop_count--;
            }
        }
        else
        {
            shoot_task.motor_info[motor_index].loop_count = 0;
        }
    }

    shoot_task.motor_info[motor_index].out_angle = shoot_task.motor_info[motor_index].rotor_angle +
                                                   abs(shoot_task.motor_info[motor_index].loop_count) * 8191;
    shoot_task.motor_info[motor_index].out_real_angle = shoot_task.motor_info[motor_index].out_angle *
                                                        360 / (8191 * 36.00f);

    shoot_task.motor_info[motor_index].angle_old_err = shoot_task.motor_info[motor_index].rotor_angle;
}

// 电机电流控制
void shoot_current_give()
{
    shoot_task.motor_info[0].set_current = pid_calc(&shoot_task.friction_pid[0], shoot_task.motor_info[0].rotor_speed, shoot_task.fric_speed_target[0]);
    shoot_task.motor_info[1].set_current = pid_calc(&shoot_task.friction_pid[1], shoot_task.motor_info[1].rotor_speed, shoot_task.fric_speed_target[1]);
    shoot_task.motor_info[2].set_current = pid_calc(&shoot_task.shoot_motor_pid, shoot_task.motor_info[2].rotor_speed, shoot_task.shoot_speed_target);
    set_motor_current_can2(0, shoot_task.motor_info[0].set_current, shoot_task.motor_info[1].set_current, shoot_task.motor_info[2].set_current, 0);
}

// 连发
static void shoot_brust(void)
{
    shoot_task.fric_speed_target[0] = -200;
    shoot_task.fric_speed_target[1] = 200;
    shoot_task.shoot_speed_target = 800;

    // 电机电流控制
    shoot_current_give();
}

// 停止发射
static void stop_shoot(void)
{
    // 电机速度与遥控器通道的对应关系
    shoot_task.fric_speed_target[0] = 0;    shoot_task.fric_speed_target[1] = 0;
    shoot_task.shoot_speed_target = 0;

    // 电机电流控制
    shoot_current_give();
}

// 单发
static void shoot_single(void)
{
    // 电机速度与遥控器通道的对应关系
    shoot_task.fric_speed_target[0] = -0;
    shoot_task.fric_speed_target[1] = 200;
    shoot_task.angle_target = shoot_task.motor_info[2].rotor_angle + 200;

    pid_calc(&shoot_task.shoot_angle_pid, shoot_task.motor_info[2].rotor_angle, shoot_task.angle_target);

    shoot_task.motor_info[0].set_current = pid_calc(&shoot_task.friction_pid[0], shoot_task.motor_info[0].rotor_speed, shoot_task.fric_speed_target[0]);
    shoot_task.motor_info[1].set_current = pid_calc(&shoot_task.friction_pid[1], shoot_task.motor_info[1].rotor_speed, shoot_task.fric_speed_target[1]);
    shoot_task.motor_info[2].set_current = pid_calc(&shoot_task.shoot_motor_pid, shoot_task.motor_info[2].rotor_speed, shoot_task.shoot_angle_pid.out);
    set_motor_current_can2(0, shoot_task.motor_info[0].set_current, shoot_task.motor_info[1].set_current, shoot_task.motor_info[2].set_current, 0);
}

// 发射机构任务
void Shoot_task(void const *pvParameters)
{
    shoot_task_init();
    shoot_task.angle_target = shoot_task.motor_info[2].out_real_angle + 45;
    if (shoot_task.angle_target > 360)
    {
        shoot_task.angle_target -= 360;
    }

    for (;;)
    {
        if (rc_ctrl.rc.s[0] == 1)
        {
            LEDR_ON();
            LEDB_OFF();
            LEDG_OFF();
            shoot_brust();
        }
        else if (rc_ctrl.rc.s[0] == 2)
        {
            LEDR_OFF();
            LEDB_OFF();
            LEDG_ON();
            stop_shoot();
        }
        else if (rc_ctrl.rc.s[0] == 3)
        {
            LEDR_OFF();
            LEDB_ON();
            LEDG_OFF();
            shoot_single();
        }
        else if (shoot_flag == 0)
        {
            LEDR_ON();
            LEDB_OFF();
            LEDG_OFF();
            if (abs(shoot_task.angle_target - shoot_task.motor_info[2].out_real_angle) > 1)
            {
                shoot_brust();
            }
            else
            {
                stop_shoot();
                shoot_flag = 1;
            }
        }
        else
        {
            LEDR_OFF();
            LEDB_OFF();
            LEDG_OFF();
            stop_shoot();
        }

        osDelay(1);
    }
}