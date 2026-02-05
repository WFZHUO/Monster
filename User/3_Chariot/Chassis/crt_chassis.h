/**
 * @file crt_chassis.h
 * @author WFZ
 * @brief 底盘电控
 * @version 0.0
 * @date 2025-12-29
 *
 *
 */
/** 
    Motor[0]  左后轮
    Motor[1]  左前轮
    Motor[2]  右前轮
    Motor[3]  右后轮

    底盘坐标系：
    X轴：指向战车前方
    Y轴：指向战车左侧
    Z轴：指向战车顶部

    */

#ifndef __CRT_CHASSIS_H__
#define __CRT_CHASSIS_H__

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor.h"
#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘控制类型
 *
 */
typedef enum 
{
    Chassis_Control_State_DISABLE = 0,
    Chassis_Control_State_NORMAL,
} Enum_Chassis_Control_State;

/**
 * @brief 战车移动坐标系选择
 * @note Crt_Move_CCS 表示操作手控制战车运动的坐标系为底盘坐标系
 * @note Crt_Move_GCS 表示操作手控制战车运动的坐标系为云台坐标系
 *
 */
typedef enum 
{
    Crt_Move_CCS = 0,      
    Crt_Move_GCS       
} Crt_Move_Coordinate_System_Mode;

/**
 * @brief Specialized, 麦克纳姆底盘类
 *
 */
class Class_Chassis
{
public:

    //定义底盘四个轮子电机
    Class_Motor_C620 Motor[4];

    void Init(void);

    inline float Get_Now_Velocity_X();

    inline float Get_Now_Velocity_Y();

    inline float Get_Now_Omega();

    inline Enum_Chassis_Control_State Get_Chassis_Control_State();

    inline Crt_Move_Coordinate_System_Mode Get_Crt_Move_CS_Mode();

    inline float Get_Target_Velocity_X();

    inline float Get_Target_Velocity_Y();

    inline float Get_Target_Omega();

    inline float Get_Chassis_Max_Speed();

    inline float Get_Chassis_Max_Omega();

    inline float Get_Gimbal_Angle();

    inline void Set_Chassis_Control_State(Enum_Chassis_Control_State __Chassis_Control_State);

    inline void Set_Crt_Move_CS_Mode(Crt_Move_Coordinate_System_Mode __Crt_Move_CS_Mode);

    inline void Set_Target_Velocity_X(float __Target_Velocity_X);

    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Chassis_Max_Speed(float __Chassis_Max_Speed);

    inline void Set_Chassis_Max_Omega(float __Chassis_Max_Omega);

    inline void Set_Gimbal_Angle(float __Gimbal_Angle);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_2ms_Resolution_PeriodElapsedCallback();

    void TIM_2ms_Control_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 常量
    const float Chassis_Half_Length = 0.185f;  // 底盘长度的一半 L/2 单位m
    const float Chassis_Half_Width = 0.2f;  // 底盘宽度的一半 W/2 单位m
    const float Wheel_Radius = 0.075f; //轮子半径 单位m

    // 内部变量

    //电机目标角速度值 单位rad/s
    float Target_Wheel_Omega[4];

    // 读变量

    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 写变量

    // 读写变量

    // 底盘控制方法
    Enum_Chassis_Control_State Chassis_Control_State = Chassis_Control_State_NORMAL;
    // 战车移动坐标系选择，默认云台坐标系，符合RM默认控制模式，可以开启小陀螺，CCS更适用于RC遥控车比赛
    Crt_Move_Coordinate_System_Mode Crt_Move_CS_Mode = Crt_Move_GCS;

    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;
    //底盘最大x,y方向速度m/s
    float Chassis_Max_Speed = 0.8f; //实测0.8m/s 已是当前减速箱和电机所能在各个电机合成出的底盘行进运动姿态正常的最大速度，若不要求底盘行进时姿态正常，更追求高速，可改为3.0f    
    //底盘最大角速度rad/s
    float Chassis_Max_Omega = 4.15f;// 同理可改为 2.0f*pi

    //云台相对底盘的角度（弧度）
    float Gimbal_Angle = 0.0f;

    //内部函数

    void Self_Resolution();

    void Kinematics_Inverse_Resolution();

    void Kinematics_GimbalToChassis();

    void Output_To_Motor();

};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float Class_Chassis::Get_Now_Velocity_X()
{
    return (Now_Velocity_X);
}

/**
 * @brief 获取当前速度Y
 *
 * @return float 当前速度Y
 */
inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return (Now_Velocity_Y);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Chassis::Get_Now_Omega()
{
    return (Now_Omega);
}

/**
 * @brief 获取底盘控制状态
 *
 * @return Enum_Chassis_Control_State 底盘控制状态
 */
inline Enum_Chassis_Control_State Class_Chassis::Get_Chassis_Control_State()
{
    return (Chassis_Control_State);
}

/**
 * @brief 获取战车移动坐标系选择
 *
 * @return Crt_Move_Coordinate_System_Mode 战车移动坐标系选择
 */
inline Crt_Move_Coordinate_System_Mode Class_Chassis::Get_Crt_Move_CS_Mode()
{
    return (Crt_Move_CS_Mode);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
inline float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标速度角速度
 *
 * @return float 目标速度角速度
 */
inline float Class_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取底盘最大速度
 *
 * @return float 底盘最大速度
 */
inline float Class_Chassis::Get_Chassis_Max_Speed()
{
    return (Chassis_Max_Speed);
}

/**
 * @brief 获取底盘最大角速度
 *
 * @return float 底盘最大角速度
 */
inline float Class_Chassis::Get_Chassis_Max_Omega()
{
    return (Chassis_Max_Omega);
}

/**
 * @brief 获取云台相对底盘的角度
 *
 * @return float 云台相对底盘的角度
 */
inline float Class_Chassis::Get_Gimbal_Angle()
{
    return (Gimbal_Angle);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_State 底盘控制方法
 */
inline void Class_Chassis::Set_Chassis_Control_State(Enum_Chassis_Control_State __Chassis_Control_State)
{
    Chassis_Control_State = __Chassis_Control_State;
}

/**
 * @brief 设定战车移动坐标系选择
 *
 * @param __Crt_Move_CS_Mode 战车移动坐标系选择
 */
inline void Class_Chassis::Set_Crt_Move_CS_Mode(Crt_Move_Coordinate_System_Mode __Crt_Move_CS_Mode)
{
    Crt_Move_CS_Mode = __Crt_Move_CS_Mode;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定底盘最大速度
 *
 * @param __Chassis_Max_Speed 底盘最大速度
 */
inline void Class_Chassis::Set_Chassis_Max_Speed(float __Chassis_Max_Speed)
{
    Chassis_Max_Speed = __Chassis_Max_Speed;
}

/**
 * @brief 设定底盘最大角速度
 *
 * @param __Chassis_Max_Omega 底盘最大角速度
 */
inline void Class_Chassis::Set_Chassis_Max_Omega(float __Chassis_Max_Omega)
{
    Chassis_Max_Omega = __Chassis_Max_Omega;
}

/**
 * @brief 设定云台相对底盘的角度
 *
 * @param __Gimbal_Angle 云台相对底盘的角度
 */
inline void Class_Chassis::Set_Gimbal_Angle(float __Gimbal_Angle)
{
    Gimbal_Angle = __Gimbal_Angle;
}

#endif

/*****************************************************************************/
