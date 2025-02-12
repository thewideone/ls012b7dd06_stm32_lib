/*
 * ls012b7dd06_tim.h
 *
 *  Created on: Feb 11, 2025
 *      Author: szyme
 */

#ifndef LS012B7DD06_STM32_INC_LS012B7DD06_TIM_H_
#define LS012B7DD06_STM32_INC_LS012B7DD06_TIM_H_

#include "stm32u5xx_hal.h"
#include "ls012b7dd06_config.h"

extern TIM_HandleTypeDef *hlcd_tim_halfline; 	// for halflines
extern TIM_HandleTypeDef *hlcd_tim_delay;		// for us delays
extern TIM_HandleTypeDef *hlcd_tim_adv;		// for GSP

HAL_StatusTypeDef lcd_TIM_init(TIM_HandleTypeDef *hhalfline_tim, TIM_HandleTypeDef *hdelay_tim, TIM_HandleTypeDef *hadv_tim);
HAL_StatusTypeDef lcd_PWM_power_enable(void);
HAL_StatusTypeDef lcd_PWM_power_disable(void);

#endif /* LS012B7DD06_STM32_INC_LS012B7DD06_TIM_H_ */
