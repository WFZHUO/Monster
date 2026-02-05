/**
 * @file crt_gimbal.cpp
 * @author wfz
 * @brief 云台控制
 * @version 0.0
 * @date 2026-1-4
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    //IMU初始化
    IMU_Gimbal.Init();

    //Yaw电机初始化
    Motor_Yaw.PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, 2.0f * PI, 2.0f * PI, 2.0f * PI);
    Motor_Yaw.PID_Omega.Init(0.5f, 0.7f, 0.0f, 0.0f,
                            1.83f, 0.183f, 3.0f,
                            0.001f,
                            0.005f, //加死区防止陀螺仪零漂
                            0.0f, 0.0f, 30.0f, PID_D_First_DISABLE,
                            PID_DIRECT,
                            0.3f,
                            PID_ZPIB_ENABLE
                            );//积分限幅和积分分离阈值按照角度环实际情况修改
    //，，，积分分离阈值再量

    Motor_Yaw.Init(&hcan1,Motor_CAN_ID_0x208,Motor_Control_Method_OMEGA,-3128);

    //Pitch电机初始化
    Motor_Pitch.PID_Omega.Init(0.35f, 0.0f, 0.0f, 0.0f, 3.0f, 3.0f, 3.0f);
    Motor_Pitch.PID_Angle.Init(11.2f, 0.0f, 0.0f, 0.0f, 6.0f * PI, 6.0f * PI, 6.0f * PI);

    Motor_Pitch.Init(&hcan1,Motor_CAN_ID_0x205,Motor_Control_Method_ANGLE,204);

}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Gimbal::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Motor_Yaw.TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Pitch.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief TIM定时器中断解算回调函数
 *
 */
void Class_Gimbal::TIM_1ms_Resolution_PeriodElapsedCallback()
{
    Self_Resolution();
}

/**
 * @brief TIM定时器中断控制回调函数
 *
 */
void Class_Gimbal::TIM_1ms_Control_PeriodElapsedCallback()
{
    IMU_Gimbal.TIM_Calculate_PeriodElapsedCallback();

    Output();

    //Motor_Pitch.Set_External_Omega(IMU_Gimbal.Get_Gyro_Y());

    Motor_Yaw.TIM_Calculate_PeriodElapsedCallback();
    Motor_Pitch.TIM_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 自身解算
 *
 */
void Class_Gimbal::Self_Resolution()
{
    Now_Yaw_Angle = Motor_Yaw.Get_Now_Angle();

    // pitch轴角度归化到±PI之间
    Now_Pitch_Angle = Math_Modulus_Normalization(Motor_Pitch.Get_Now_Angle(), 2.0f * PI);

    if (Motor_Yaw.External_Omega_Active_Flag)
    {
        Now_Yaw_Omega = Motor_Yaw.Get_Now_External_Omega();
    }
    else
    {
        Now_Yaw_Omega = Motor_Yaw.Get_Now_Omega();
    }

    if (Motor_Pitch.External_Omega_Active_Flag)
    {
        Now_Pitch_Omega = Motor_Pitch.Get_Now_External_Omega();
    }
    else
    {
        Now_Pitch_Omega = Motor_Pitch.Get_Now_Omega();
    }
}

/**
 * @brief 输出到电机
 *
 */
void Class_Gimbal::Output()
{
    if (Gimbal_Control_State == Gimbal_Control_State_DISABLE)
    {
        // 云台失能
        //以后再说
    }
    else if (Gimbal_Control_State == Gimbal_Control_State_NORMAL)
    {
        if(Motor_Yaw.Get_Control_Method() == Motor_Control_Method_ANGLE || Motor_Pitch.Get_Control_Method() == Motor_Control_Method_ANGLE){
            Motor_Nearest_Transposition();
        }
        
        if(Motor_Yaw.Get_Control_Method() == Motor_Control_Method_ANGLE){
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        }else{
            Motor_Yaw.Set_Target_Omega(Target_Yaw_Omega);
        }

        if(Motor_Pitch.Get_Control_Method() == Motor_Control_Method_ANGLE){
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        }else{
            Motor_Pitch.Set_Target_Omega(Target_Pitch_Omega);
        }
    }
}

/**
 * @brief Yaw电机就近转位和Pitch电机限位
 *
 */
void Class_Gimbal::Motor_Nearest_Transposition()
{
    float tmp_delta_angle;

    // Yaw就近转位
    tmp_delta_angle = fmod(Target_Yaw_Angle - Now_Yaw_Angle, 2.0f * PI);
    if (tmp_delta_angle > PI)
    {
        tmp_delta_angle -= 2.0f * PI;
    }
    else if (tmp_delta_angle < -PI)
    {
        tmp_delta_angle += 2.0f * PI;
    }
    //Target_Yaw_Angle = Motor_Yaw.Get_Now_Angle() + tmp_delta_angle;
    Target_Yaw_Angle = Now_Yaw_Angle + tmp_delta_angle;

    // Pitch电机限位
    Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
    tmp_delta_angle = Target_Pitch_Angle - Now_Pitch_Angle;
    //Target_Pitch_Angle = Motor_Pitch.Get_Now_Angle() + tmp_delta_angle;
    Target_Pitch_Angle = Now_Pitch_Angle + tmp_delta_angle;

}



