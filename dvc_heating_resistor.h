/**
 * @file dvc_heating_resistor.h
 * @author WFZ
 * @brief 加热电阻驱动
 * @version 0.0
 * @date 2026-02-05
 *
 */

#ifndef DVC_HEATING_RESISTOR_H
#define DVC_HEATING_RESISTOR_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "alg_pid.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 加热电阻类
 *
 */
class Class_Heating_Resistor
{
public:

    //note:只能有P项,不能加I项,D项
    Class_PID PID_Temperature;

    bool Temperature_is_OK = false;

    void Init(float __Output_Duty_Max = 1000);

    inline void Set_Target_Temperature(float temperature);

    inline void Set_Current_Temperature(float temperature);

    inline void Set_Output_Duty(float __Output_Duty);

    inline float Get_Target_Temperature(void);

    inline float Get_Current_Temperature(void);

    inline float Get_Output_Duty(void);

    void TIM_Calculate_PeriodElapsedCallback();

protected:

    float Target_Temperature = 50.0f;

    float Current_Temperature;

    float Output_Duty;

    float Output_Duty_Max = 1000;

    void PID_Calculate();
    void SetDuty();
};

/* Exported variables --------------------------------------------------------*/

extern TIM_HandleTypeDef htim10;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 设置加热电阻目标温度
 *
 * @param temperature 目标温度
 */
inline void Class_Heating_Resistor::Set_Target_Temperature(float temperature)
{
    Target_Temperature = temperature;
}

/**
 * @brief 设置加热电阻当前温度
 *
 * @param temperature 当前温度
 */
inline void Class_Heating_Resistor::Set_Current_Temperature(float temperature)
{
    Current_Temperature = temperature;
}

/**
 * @brief 设置加热电阻输出占空比
 *
 * @param __Output_Duty 输出占空比
 */
inline void Class_Heating_Resistor::Set_Output_Duty(float __Output_Duty)
{
    Output_Duty = __Output_Duty;
}

/**
 * @brief 获取加热电阻目标温度
 *
 * @return float 目标温度
 */
inline float Class_Heating_Resistor::Get_Target_Temperature(void)
{
    return Target_Temperature;
}

/**
 * @brief 获取加热电阻当前温度
 *
 * @return float 当前温度
 */
inline float Class_Heating_Resistor::Get_Current_Temperature(void)
{
    return Current_Temperature;
}

/**
 * @brief 获取加热电阻输出占空比
 *
 * @return float 输出占空比
 */
inline float Class_Heating_Resistor::Get_Output_Duty(void)
{
    return Output_Duty;
}

#endif

/********************************************************************/
