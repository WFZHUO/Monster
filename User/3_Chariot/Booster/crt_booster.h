/**
 * @file crt_booster.h
 * @author WangFZhuo
 * @brief 发射机构电控
 * @version 0.0
 * @date 2026-01-10
 *
 *
 */

/**
 * @brief 摩擦轮CAN ID
 * 1 2
 */

 /**
 * @brief 拨弹盘CAN ID
 * 3
 */

#ifndef CRT_BOOSTER_H
#define CRT_BOOSTER_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor.h"
#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 发射机构控制类型
 *
 */
typedef enum
{
    Booster_Control_Type_DISABLE = 0,  // 全停（摩擦停 + 拨弹停）
    Booster_Control_Type_CEASEFIRE,    // 预热（摩擦转 + 拨弹停）
    Booster_Control_Type_SPOT,         // 单发（拨弹走一格，立刻回到 CEASEFIRE）
    Booster_Control_Type_AUTO,         // 连发（拨弹恒速）
} Enum_Booster_Control_Type;

/**
 * @brief 发射机构类
 *
 */
class Class_Booster
{
public:

    //拨弹盘电机
    Class_Motor_C610 Motor_Driver;

    //左摩擦轮电机
    Class_Motor_C620 Motor_Friction_Left;

    //右摩擦轮电机
    Class_Motor_C620 Motor_Friction_Right;

    void Init(); 

    inline Enum_Booster_Control_Type Get_Booster_Control_Type() const { return Control_Type; }

    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type t) { Control_Type = t; }

    inline void Set_Friction_Omega(float omega_radps) { Friction_Omega = omega_radps; }

    inline void Set_Auto_Frequency(float freq_hz) { Auto_Ammo_Frequency = freq_hz; }
    
    inline void Set_Ammo_Num_Per_Round(float n) { Ammo_Num_Per_Round = n; }

    // 触发一次单发（你在遥控器逻辑里做“边沿触发”时调用）
    void Trigger_Spot();

    // 放到 1ms 任务里调用
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    // 100ms 调一次（可选，但建议上，掉线会清积分）
    void TIM_100ms_Alive_PeriodElapsedCallback();

    // 方向可调：如果你发现转向反了，改这里符号最省事
    int8_t Driver_Dir = -1;
    int8_t Friction_Left_Dir = +1;
    int8_t Friction_Right_Dir = -1;

protected:
    // 拨弹盘一圈子弹数
    float Ammo_Num_Per_Round = 7.0f;         

    // 摩擦轮角速度（rad/s）
    float Friction_Omega = 830.0f;   //弹丸速度 = 830 * 0.03 m/s = 24.9 m/s

    float Auto_Ammo_Frequency = 15.0f;       // 连发频率（发/秒），先给个保守值

    // 发射机构状态
    Enum_Booster_Control_Type Control_Type = Booster_Control_Type_DISABLE;

    // 单发状态
    bool Spot_Busy = false;
    float Spot_Target_Angle = 0.0f;
    uint16_t Spot_Timeout_ms = 0;

    //内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/



#endif

/**************************************************************************************/
