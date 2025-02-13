/*
 * ls012b7dd06_tim.c
 *
 *  Created on: Feb 11, 2025
 *      Author: szyme
 */

#include <ls012b7dd06_tim.h~>
#include "tim.h"	// for the halfline timer from the main project (Core/Inc/tim.h)
#include <stdio.h>	// for sprintf()

TIM_HandleTypeDef *hlcd_tim_halfline; 	// for halflines
TIM_HandleTypeDef *hlcd_tim_delay;		// for us delays
TIM_HandleTypeDef *hlcd_tim_adv;		// for GSP
TIM_HandleTypeDef *hlcd_tim_pwr;		// for VA, VB and VCOM

void lcd_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void lcd_TIM_PWM_PulseFinishedCallback_GSP(TIM_HandleTypeDef *htim);
void lcd_TIM_PWM_PulseFinishedCallback_halfline(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef lcd_TIM_init(TIM_HandleTypeDef *htim_halfline, TIM_HandleTypeDef *htim_delay,
								TIM_HandleTypeDef *htim_adv, TIM_HandleTypeDef *htim_pwr){
	if (htim_halfline == NULL || htim_delay == NULL || htim_adv == NULL || htim_pwr == NULL)
		return HAL_ERROR;

	hlcd_tim_halfline = htim_halfline;
	hlcd_tim_delay = htim_delay;
	hlcd_tim_adv = htim_adv;
	hlcd_tim_pwr = htim_pwr;

	HAL_StatusTypeDef ret;

	ret = HAL_TIM_RegisterCallback(hlcd_tim_delay, HAL_TIM_PERIOD_ELAPSED_CB_ID, lcd_TIM_PeriodElapsedCallback);
	if( ret != HAL_OK ){
	  Error_Handler();
    }

	ret = HAL_TIM_RegisterCallback(hlcd_tim_adv, HAL_TIM_PWM_PULSE_FINISHED_CB_ID, lcd_TIM_PWM_PulseFinishedCallback_GSP);
	if( ret != HAL_OK ){
	  Error_Handler();
	}

	ret = HAL_TIM_RegisterCallback(hlcd_tim_halfline, HAL_TIM_PWM_PULSE_FINISHED_CB_ID, lcd_TIM_PWM_PulseFinishedCallback_halfline);
	if( ret != HAL_OK ){
	  Error_Handler();
	}


	return HAL_OK;
}



