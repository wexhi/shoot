#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

// 电机信息结构体
typedef struct
{
    uint16_t can_id;        // ID号
    int16_t set_current;    // 发送信息
    uint16_t rotor_angle;   // 现在的机械刻度角度
    int16_t rotor_speed;    // 现在的转速
    int16_t torque_current; // 实际转矩电流
    uint8_t temp;           // 电机温度
    fp32 real_angle;        // 实际角度
    fp32 out_angle;         // 输出机械刻度角度
    fp32 out_real_angle;    // 输出实际角度
    fp32 angle_error;       // 角度误差
    fp32 angle_old_err;     // 上次角度误差
    int8_t loop_count;     // 转圈数
} motor_info_t;

#endif
