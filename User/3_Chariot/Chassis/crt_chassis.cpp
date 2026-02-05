/**
 * @file crt_chassis.cpp
 * @author WFZ
 * @brief 底盘电控
 * @version 0.0
 * @date 2025-12-29
 *
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘初始化
 *
 */
void Class_Chassis::Init(void)
{
    Motor[0].Init(&hcan1,Motor_CAN_ID_0x201,Motor_Control_Method_OMEGA);
	Motor[0].PID_Omega.Init(0.73242f, 0.0f, 0.0f, 0.0f);
	
	Motor[1].Init(&hcan1,Motor_CAN_ID_0x202,Motor_Control_Method_OMEGA);
	Motor[1].PID_Omega.Init(0.73242f, 0.0f, 0.0f, 0.0f);
	
	Motor[2].Init(&hcan1,Motor_CAN_ID_0x203,Motor_Control_Method_OMEGA);
	Motor[2].PID_Omega.Init(0.73242f, 0.0f, 0.0f, 0.0f);
	
	Motor[3].Init(&hcan1,Motor_CAN_ID_0x204,Motor_Control_Method_OMEGA);
	Motor[3].PID_Omega.Init(0.73242f, 0.0f, 0.0f, 0.0f);

    // 战车移动坐标系选择，默认云台坐标系
    Crt_Move_CS_Mode = Crt_Move_GCS;
}

/**
 * @brief TIM定时器中断定期检测底盘各电机是否存活
 *
 */
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback(void)
{
    for (int i = 0; i < 4; i++)
    {
        Motor[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
}

/**
 * @brief TIM定时器中断解算回调函数
 *
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback(void)
{
    Self_Resolution();
}

/**
 * @brief TIM定时器中断控制回调函数
 *
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
    Kinematics_Inverse_Resolution();

    Output_To_Motor();
}

/**
 * @brief 自身解算，运动学正解。根据四个轮子的实际转速计算底盘的实际速度和角速度
 *
 */
void Class_Chassis::Self_Resolution(void)
{
    Now_Velocity_X = 0.0f;
    Now_Velocity_Y = 0.0f;
    Now_Omega = 0.0f;

    float wheel_omega[4];
    for (int i = 0; i < 4; i++)
        wheel_omega[i] = Motor[i].Get_Now_Omega();

    float wheel_speeds[4];
    for (int i = 0; i < 4; i++)
        wheel_speeds[i] = wheel_omega[i] * Wheel_Radius;

    // 右侧轮电机安装方向与左侧相反：这里把实测值反号回“轮子线速度”物理方向
    wheel_speeds[2] = -wheel_speeds[2];  // 右前
    wheel_speeds[3] = -wheel_speeds[3];  // 右后

    // 旋转半径
    float R = Chassis_Half_Length + Chassis_Half_Width;

    // 统一的麦轮模型（前x 左y 上z，+omega 逆时针）：
    // v_LR(0)= vx + vy - w*R
    // v_LF(1)= vx - vy - w*R
    // v_RF(2)= vx + vy + w*R
    // v_RR(3)= vx - vy + w*R

    Now_Velocity_X = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / 4.0f;
    Now_Velocity_Y = (wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2] - wheel_speeds[3]) / 4.0f;
    Now_Omega      = (-wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / (4.0f * R);
}


/**
 * @brief 运动学逆解算
 *
 */
void Class_Chassis::Kinematics_Inverse_Resolution(void)
{
    float wheel_speeds[4];

    // 根据模式选择坐标系
    if (Crt_Move_CS_Mode == Crt_Move_GCS){
        Kinematics_GimbalToChassis();
    }

    Math_Constrain(&Target_Velocity_X, -Chassis_Max_Speed, Chassis_Max_Speed);
    Math_Constrain(&Target_Velocity_Y, -Chassis_Max_Speed, Chassis_Max_Speed);
    Math_Constrain(&Target_Omega,      -Chassis_Max_Omega, Chassis_Max_Omega);

    // 旋转半径
    float R = Chassis_Half_Length + Chassis_Half_Width;

    // 统一的麦轮逆解（前x 左y 上z，+omega 逆时针）
    // 0:左后(LR) 1:左前(LF) 2:右前(RF) 3:右后(RR)
    wheel_speeds[0] = Target_Velocity_X + Target_Velocity_Y - Target_Omega * R; // LR
    wheel_speeds[1] = Target_Velocity_X - Target_Velocity_Y - Target_Omega * R; // LF
    wheel_speeds[2] = Target_Velocity_X + Target_Velocity_Y + Target_Omega * R; // RF
    wheel_speeds[3] = Target_Velocity_X - Target_Velocity_Y + Target_Omega * R; // RR

    // 右侧轮电机方向相反：对下发目标取反，保轮子线速与模型一致
    wheel_speeds[2] = -wheel_speeds[2];
    wheel_speeds[3] = -wheel_speeds[3];

    Target_Wheel_Omega[0] = wheel_speeds[0] / Wheel_Radius;
    Target_Wheel_Omega[1] = wheel_speeds[1] / Wheel_Radius;
    Target_Wheel_Omega[2] = wheel_speeds[2] / Wheel_Radius;
    Target_Wheel_Omega[3] = wheel_speeds[3] / Wheel_Radius;
}



/**
 * @brief 云台坐标系到底盘坐标系的转换函数
 * 
 */
void Class_Chassis::Kinematics_GimbalToChassis(void)
{
    // v_chassis = R(theta) * v_gimbal
    const float x_g = Target_Velocity_X;
    const float y_g = Target_Velocity_Y;

    Target_Velocity_X = x_g * cosf(Gimbal_Angle) - y_g * sinf(Gimbal_Angle);
    Target_Velocity_Y = x_g * sinf(Gimbal_Angle) + y_g * cosf(Gimbal_Angle);
}



/**
 * @brief 输出到电机
 *
 */
void Class_Chassis::Output_To_Motor()
{
    switch (Chassis_Control_State)
    {
    case (Chassis_Control_State_DISABLE):
    {
        // 底盘失能
        for (int i = 0; i < 4; i++)
        {
            Motor[i].Set_Control_Method(Motor_Control_Method_CURRENT);

            Motor[i].PID_Omega.Set_Integral_Error(0.0f);

            Motor[i].Set_Target_Current(0.0f);

            Motor[i].Set_Feedforward_Current(0.0f);
        }

        break;
    }
    case (Chassis_Control_State_NORMAL):
    {
        for (int i = 0; i < 4; i++)
        {
            Motor[i].Set_Control_Method(Motor_Control_Method_OMEGA);
        }

        for (int i = 0; i < 4; i++)
        {
            Motor[i].Set_Target_Omega(Target_Wheel_Omega[i]);
        }

        break;
    }
    }

    for (int i = 0; i < 4; i++)
    {
        Motor[i].TIM_Calculate_PeriodElapsedCallback();
    }
}

/*****************************************************************************/
