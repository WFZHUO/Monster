/**
 * @file dvc_buzzer.h
 * @author wfz
 * @version 0.0
 * @date 2026-1-4
 *
 */

#ifndef __DVC_BUZZER_H__
#define __DVC_BUZZER_H__

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim4;

void dvc_buzzer_init(void);
void dvc_buzzer_SetOn(void);
void dvc_buzzer_SetOff(void);

#endif
