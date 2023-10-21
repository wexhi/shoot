
#include "shoot_task.h"
#include "cmsis_os.h"
#include "drv_can.h"

extern RC_ctrl_t rc_ctrl;

motor_info_t shoot_motor_info[3];        // 电机信息结构体
fp32 speed_motor_pid[3] = {30, 0.5, 10}; // 速度环pid
fp32 shoot_angle_pid[3] = {30, 0.5, 10}; // 角度环pid
volatile int16_t motor_speed_target[3];

pid_struct_t shoot_motor_pid[3]; // 电机pid结构体
pid_struct_t friction_pid;       // 拨盘电机pid结构体

// 电机电流控制
void shoot_current_give()
{

    uint8_t i = 0;

    for (i = 0; i < 2; i++)
    {
        shoot_motor_info[i].set_current = pid_calc(&shoot_motor_pid[i], shoot_motor_info[i].rotor_speed, motor_speed_target[i]);
    }
    set_motor_current_can2(0, shoot_motor_info[0].set_current, shoot_motor_info[1].set_current, shoot_motor_info[2].set_current, 0);
}

// 连发
static void shoot_brust(void)
{
    // 电机速度与遥控器通道的对应关系
    motor_speed_target[0] = -200;
    motor_speed_target[1] = 200;
    motor_speed_target[2] = 200;

    // 电机电流控制
    shoot_current_give();
}

// 停止发射
static void stop_shoot(void)
{
    // 电机速度与遥控器通道的对应关系
    motor_speed_target[0] = 0;
    motor_speed_target[1] = 0;
    motor_speed_target[2] = 0;

    // 电机电流控制
    shoot_current_give();
}

// 单发
static void shoot_single(void)
{
    // 电机速度与遥控器通道的对应关系
    motor_speed_target[0] = 0;
    motor_speed_target[1] = 0;
}

void Shoot_task(void const *pvParameters)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        pid_init(&shoot_motor_pid[i], speed_motor_pid, 6000, 6000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
    }
    pid_init(&friction_pid, shoot_angle_pid, 6000, 6000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384

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
        else
        {
            stop_shoot();
        }

        osDelay(1);
    }
}