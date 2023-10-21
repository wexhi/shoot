#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

#define RC_MAX 660
#define RC_MIN -660
#define motor_max 900
#define motor_min -900

pid_struct_t motor_pid_shoot[3];
pid_struct_t friction_pid; // 拨盘电机pid结构体
pid_struct_t supercap_pid;
int16_t targetAngle;
motor_info_t motor_info_shoot[8];        // 电机信息结构体
fp32 shoot_motor_pid[3] = {30, 0.5, 10}; // 速度环pid
fp32 shoot_angle_pid[3] = {30, 0.5, 10}; // 角度环pid
fp32 superpid[3] = {120, 0.1, 0};
volatile int16_t Vx = 0, Vy = 0, Wz = 0;
int16_t Temp_Vx;
int16_t Temp_Vy;
int fllowflag = 0;
volatile int16_t motor_speed_target[4];
extern RC_ctrl_t rc_ctrl;
extern float powerdata[4];
extern uint16_t shift_flag;

uint8_t rc[18];
uint8_t motor_flag[4] = {1, 1, 1, 1}; // LF RF RB LB
int16_t avg_speed = 0;
// Save imu data

int chassis_mode_flag = 0;

#define angle_valve 5
#define angle_weight 55

static void shoot_brust(void);
static void shoot_single(void);
static void stop_shoot(void);

void Shoot_task(void const *pvParameters)
{
  for (uint8_t i = 0; i < 3; i++)
  {
    pid_init(&motor_pid_shoot[i], shoot_motor_pid, 6000, 6000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
  }
  pid_init(&friction_pid, shoot_angle_pid, 6000, 6000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384

  for (;;)
  {
    // if (rc_ctrl.rc.s[0] == 1)
    // {
    //   LEDR_ON();
    //   LEDB_OFF();
    //   LEDG_OFF();
    //   shoot_brust();
    // }
    // else if (rc_ctrl.rc.s[0] == 2)
    // {
    //   LEDR_OFF();
    //   LEDB_OFF();
    //   LEDG_ON();
    //   stop_shoot();
    // }
    // else if (rc_ctrl.rc.s[0] == 3)
    // {
    //   LEDR_OFF();
    //   LEDB_ON();
    //   LEDG_OFF();
    //   shoot_single();
    // }
    // else
    // {
    //   stop_shoot();
    // }

    osDelay(1);
  }
}

// 速度限制函数
void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed)
{
  uint8_t i = 0;
  int16_t max = 0;
  int16_t temp = 0;
  int16_t max_speed = limit_speed;
  fp32 rate = 0;
  for (i = 0; i < 4; i++)
  {
    temp = (motor_speed[i] > 0) ? (motor_speed[i]) : (-motor_speed[i]); // 求绝对值

    if (temp > max)
    {
      max = temp;
    }
  }

  if (max > max_speed)
  {
    rate = max_speed * 1.0 / max; //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
    for (i = 0; i < 4; i++)
    {
      motor_speed[i] *= rate;
    }
  }
}
// 电机电流控制
void chassis_current_give()
{

  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    motor_info_shoot[i].set_current = pid_calc(&motor_pid_shoot[i], motor_info_shoot[i].rotor_speed, motor_speed_target[i]);
  }
  set_motor_current_can2(0, motor_info_shoot[0].set_current, motor_info_shoot[1].set_current, motor_info_shoot[2].set_current, 0);
}

// 连发
static void shoot_brust(void)
{
  // 电机速度与遥控器通道的对应关系
  motor_speed_target[0] = -200;
  motor_speed_target[1] = 200;
  motor_speed_target[2] = 200;

  // 电机电流控制
  chassis_current_give();
}

// 停止发射
static void stop_shoot(void)
{
  // 电机速度与遥控器通道的对应关系
  motor_speed_target[0] = 0;
  motor_speed_target[1] = 0;
  motor_speed_target[2] = 0;

  // 电机电流控制
  chassis_current_give();
}

static void shoot_single(void)
{
  // 电机速度与遥控器通道的对应关系
  motor_speed_target[0] = 0;
  motor_speed_target[1] = 0;
  targetAngle = motor_info_shoot[2].rotor_angle - 1000;
  if (targetAngle <= 0 || targetAngle >= 6000)
  {
    targetAngle = 0;
  }

  // 电机电流控制
  motor_info_shoot[0].set_current = pid_calc(&motor_pid_shoot[0], motor_info_shoot[0].rotor_speed, motor_speed_target[0]);
  motor_info_shoot[1].set_current = pid_calc(&motor_pid_shoot[1], motor_info_shoot[1].rotor_speed, motor_speed_target[1]);

  // 角度控制
  // motor_info_shoot[2].set_current = pid_calc(&motor_pid_shoot[2], motor_info_shoot[2].rotor_speed, targetAngle);

  set_motor_current_can2(0, motor_info_shoot[0].set_current, motor_info_shoot[1].set_current, motor_info_shoot[2].set_current, 0);
}

// 线性映射函数
static int16_t map_range(int value, int from_min, int from_max, int to_min, int to_max)
{
  // 首先将输入值映射到[0, 1]的范围
  double normalized_value = (value * 1.0 - from_min * 1.0) / (from_max * 1.0 - from_min * 1.0);

  // 然后将[0, 1]的范围映射到[to_min, to_max]的范围
  int16_t mapped_value = (int16_t)(normalized_value * (to_max - to_min) + to_min);

  return mapped_value;
}

void RC_to_motor(void)
{
  // 电机速度与遥控器通道的对应关系
  avg_speed = map_range(rc_ctrl.rc.ch[3], RC_MIN, RC_MAX, motor_min, motor_max);
  motor_speed_target[CHAS_LF] = avg_speed;
  motor_speed_target[CHAS_RF] = -avg_speed;
  motor_speed_target[CHAS_RB] = -avg_speed;
  motor_speed_target[CHAS_LB] = avg_speed;

  // 判断需要旋转的电机
  for (uint8_t i = 0; i < 4; i++)
  {
    /* code */
    if (motor_flag[i] == 0)
    {
      motor_speed_target[i] = 0;
    }
  }
  // 电机电流控制
  chassis_current_give();
}

void test_motor(int16_t avg)
{
  // 电机速度与遥控器通道的对应关系
  motor_speed_target[CHAS_LF] = avg;
  motor_speed_target[CHAS_RF] = -avg;
  motor_speed_target[CHAS_RB] = -avg;
  motor_speed_target[CHAS_LB] = avg;

  // 判断需要旋转的电机
  for (uint8_t i = 0; i < 4; i++)
  {
    /* code */
    if (motor_flag[i] == 0)
    {
      motor_speed_target[i] = 0;
    }
  }
  // 电机电流控制
  chassis_current_give();
}

void RC_Move(void)
{
  // 从遥控器获取控制输入
  // int16_t forward_backward_input = rc_ctrl.rc.ch[1]; // 前后输入
  // int16_t left_right_input = rc_ctrl.rc.ch[0];       // 左右输入
  // int16_t rotation_input = rc_ctrl.rc.ch[2];         // 旋转输入
  Vx = rc_ctrl.rc.ch[3]; // 前后输入
  Vy = rc_ctrl.rc.ch[2]; // 左右输入
  Wz = rc_ctrl.rc.ch[0]; // 旋转输入

  /*************记得加上线性映射***************/
  Vx = map_range(Vx, RC_MIN, RC_MAX, motor_min, motor_max);
  Vy = map_range(Vy, RC_MIN, RC_MAX, motor_min, motor_max);
  Wz = map_range(Wz, RC_MIN, RC_MAX, motor_min, motor_max);

  // 根据分解的速度调整电机速度目标
  // motor_speed_target[CHAS_LF] = Vx - Vy - Wz;
  // motor_speed_target[CHAS_RF] = Vx + Vy + Wz;
  // motor_speed_target[CHAS_RB] = Vx - Vy + Wz;
  // motor_speed_target[CHAS_LB] = Vx + Vy - Wz;
  motor_speed_target[CHAS_LF] = Wz + Vx + Vy;
  motor_speed_target[CHAS_RF] = Wz - Vx + Vy;
  motor_speed_target[CHAS_RB] = Wz - Vx - Vy;
  motor_speed_target[CHAS_LB] = Wz + Vx - Vy;

  // 电机电流控制
  chassis_current_give();
}