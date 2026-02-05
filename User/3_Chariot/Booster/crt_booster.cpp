/**
 * @file crt_booster.cpp
 * @author WangFZhuo
 * @brief 发射机构电控
 * @version 0.0
 * @date 2026-01-10
 *
 *
 */

/**
 * @brief 摩擦轮编号
 * 1 2
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{
    //拨弹盘
    Motor_Driver.PID_Angle.Init(50.0f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f,20.0f ); 
    Motor_Driver.PID_Omega.Init(3.40f, 200.0f, 0.0f, 0.0f,10.0f, 10.0f, 10.0f);

    Motor_Driver.Init(&hcan2, Motor_CAN_ID_0x203, Motor_Control_Method_OMEGA);
    
    //摩擦轮
    Motor_Friction_Left.PID_Omega.Init(0.15f, 0.3f, 0.002f, 0.0f,20.0f, 20.0f, 20.0f);
    Motor_Friction_Left.Init(&hcan2, Motor_CAN_ID_0x201, Motor_Control_Method_OMEGA, 1);
    
    Motor_Friction_Right.PID_Omega.Init(0.15f, 0.3f, 0.002f, 0.0f,20.0f, 20.0f, 20.0f);
    Motor_Friction_Right.Init(&hcan2, Motor_CAN_ID_0x202, Motor_Control_Method_OMEGA,1);
    

    Control_Type = Booster_Control_Type_DISABLE;
}

void Class_Booster::Trigger_Spot()
{
    // 正在打一发就别重复触发
    if (Spot_Busy) return;

    const float step_angle = 2.0f * PI / Ammo_Num_Per_Round;

    Spot_Busy = true;
    //Spot_Timeout_ms = 0;

    // 用 Driver_Dir 控制方向（你现在是写死减 step_angle）
    Spot_Target_Angle = Motor_Driver.Get_Now_Angle() - step_angle;

    Control_Type = Booster_Control_Type_SPOT;
}

void Class_Booster::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Motor_Driver.TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Friction_Left.TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_100ms_Alive_PeriodElapsedCallback();
}

void Class_Booster::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    Output();

    Motor_Driver.TIM_Calculate_PeriodElapsedCallback();
    Motor_Friction_Left.TIM_Calculate_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_Calculate_PeriodElapsedCallback();
}

void Class_Booster::Output()
{
    const float step_angle = 2.0f * PI / Ammo_Num_Per_Round;                 // 单发拨一格
    const float driver_auto_omega = Auto_Ammo_Frequency * step_angle;        // 连发拨盘角速度(rad/s)

    switch (Control_Type)
    {
    case Booster_Control_Type_DISABLE:
    {
        // 全停：摩擦=0，拨弹=0
        Motor_Driver.Set_Control_Method(Motor_Control_Method_CURRENT);
        Motor_Friction_Left.Set_Control_Method(Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_Control_Method(Motor_Control_Method_OMEGA);

        Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Left.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Right.PID_Omega.Set_Integral_Error(0.0f);

        Motor_Driver.Set_Target_Current(0.0f);
        Motor_Friction_Left.Set_Target_Omega(0.0f);
        Motor_Friction_Right.Set_Target_Omega(0.0f);
    }
    break;

    case Booster_Control_Type_CEASEFIRE:
    {
        // 预热：摩擦转，拨弹停
        Motor_Driver.Set_Control_Method(Motor_Control_Method_OMEGA);
        Motor_Friction_Left.Set_Control_Method(Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_Control_Method(Motor_Control_Method_OMEGA);

        Motor_Driver.Set_Target_Omega(0.0f);
        Motor_Friction_Left.Set_Target_Omega(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega(-Friction_Omega);
    }
    break;

    case Booster_Control_Type_SPOT:
    {
        // 单发：拨弹盘角度 += step，然后回到 CEASEFIRE
        Motor_Driver.Set_Control_Method(Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_Control_Method(Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_Control_Method(Motor_Control_Method_OMEGA);

        Motor_Friction_Left.Set_Target_Omega(Friction_Left_Dir * Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega(Friction_Right_Dir * Friction_Omega);

        Motor_Driver.Set_Target_Angle(Spot_Target_Angle);

        // ===== 到位判定（建议：角度误差 + 速度都小）=====
        float err = Spot_Target_Angle - Motor_Driver.Get_Now_Angle();
        float err_abs = Math_Abs(err);

        float angle_eps = 0.03f;   // ~1.7deg，按你机构精度调整

        if (err_abs < angle_eps)
        {
            Spot_Busy = false;
            Control_Type = Booster_Control_Type_CEASEFIRE;
        }
    }
    break;

    case Booster_Control_Type_AUTO:
    {
        // 连发：摩擦转 + 拨弹恒速
        Motor_Driver.Set_Control_Method(Motor_Control_Method_OMEGA);
        Motor_Friction_Left.Set_Control_Method(Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_Control_Method(Motor_Control_Method_OMEGA);

        Motor_Friction_Left.Set_Target_Omega(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega(-Friction_Omega);
        Motor_Driver.Set_Target_Omega(-driver_auto_omega);
    }
    break;
    }
}

/**************************************************************************************/
