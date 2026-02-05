/**
 * @file dvc_buzzer.c
 * @author wfz
 * @version 0.0
 * @date 2026-1-4
 *
 */

#include "dvc_buzzer.h"

void dvc_buzzer_init(void)
{
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
}

void dvc_buzzer_SetOn()
{
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,500);
}

void dvc_buzzer_SetOff()
{
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
}
