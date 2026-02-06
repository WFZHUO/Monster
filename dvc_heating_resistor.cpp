/**
 * @file dvc_heating_resistor.cpp
 * @author WFZ
 * @brief 加热电阻驱动
 * @version 0.0
 * @date 2026-02-05
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_heating_resistor.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 加热电阻初始化
 *
 * @param __Output_Duty_Max 输出占空比最大值
 */
void Class_Heating_Resistor::Init(float __Output_Duty_Max)
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,0);
    Output_Duty_Max = __Output_Duty_Max;
}

/**
 * @brief 周期回调函数
 */
void Class_Heating_Resistor::TIM_Calculate_PeriodElapsedCallback()
{
	PID_Calculate();

	Math_Constrain(&Output_Duty, 0.0f, Output_Duty_Max);

	SetDuty();
}

/**
 * @brief PID计算
 */
void Class_Heating_Resistor::PID_Calculate(void)
{
    PID_Temperature.Set_Target(Target_Temperature);
	PID_Temperature.Set_Now(Current_Temperature);
	PID_Temperature.TIM_Adjust_PeriodElapsedCallback();

	Output_Duty = PID_Temperature.Get_Out();
}

/**
 * @brief 设置占空比
 */
void Class_Heating_Resistor::SetDuty(void)
{
	unsigned int duty = (unsigned int)Output_Duty;
	__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,duty);
}

/********************************************************************/
