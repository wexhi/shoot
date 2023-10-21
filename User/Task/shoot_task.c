
#include "shoot_task.h"
#include "cmsis_os.h"
#include "drv_can.h"

extern RC_ctrl_t rc_ctrl;

shoot_task_t shoot_task;

// 发射机构初始化
void shoot_task_init(void)
{
    shoot_task.speed_motor_pid[0] = 30;  // 速度环pid->kp
    shoot_task.speed_motor_pid[1] = 0.5; // 速度环pid->ki
    shoot_task.speed_motor_pid[2] = 10;  // 速度环pid->kd
    shoot_task.angle_pid[0] = 30;  // 角度环pid->kp
    shoot_task.angle_pid[1] = 0.5; // 角度环pid->ki
    shoot_task.angle_pid[2] = 10;  // 角度环pid->kd
    shoot_task.angle_target = 0;         // 角度环目标值
    shoot_task.shoot_speed_target = 0;   // 拨盘速度环目标值
    shoot_task.fric_speed_target[0] = 0; // 摩擦轮电机目标值
    shoot_task.fric_speed_target[1] = 0; // 摩擦轮电机目标值

    pid_init(&shoot_task.shoot_angle_pid, shoot_task.angle_pid, 2.0f * PI, 2.0f * PI); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
    pid_init(&shoot_task.shoot_motor_pid, shoot_task.speed_motor_pid, 6000, 6000);          // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
    pid_init(&shoot_task.friction_pid[0], shoot_task.speed_motor_pid, 6000, 6000);     // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
    pid_init(&shoot_task.friction_pid[1], shoot_task.speed_motor_pid, 6000, 6000);     // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
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
    // 电机速度与遥控器通道的对应关系
    shoot_task.fric_speed_target[0] = -200;
    shoot_task.fric_speed_target[1] = 200;
    shoot_task.shoot_speed_target = 200;

    // 电机电流控制
    shoot_current_give();
}

// 停止发射
static void stop_shoot(void)
{
    // 电机速度与遥控器通道的对应关系
    shoot_task.fric_speed_target[0] = 0;
    shoot_task.fric_speed_target[1] = 0;
    shoot_task.shoot_speed_target = 0;

    // 电机电流控制
    shoot_current_give();
}

// 单发
static void shoot_single(void)
{
    // 电机速度与遥控器通道的对应关系
    shoot_task.fric_speed_target[0] = -200;
    shoot_task.fric_speed_target[1] = 200;
    
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