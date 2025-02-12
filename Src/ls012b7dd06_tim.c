/*
 * ls012b7dd06_tim.c
 *
 *  Created on: Feb 11, 2025
 *      Author: szyme
 */

#include "ls012b7dd06_tim.h"
#include "tim.h"	// for the halfline timer from the main project (Core/Inc/tim.h)

TIM_HandleTypeDef *hlcd_tim_halfline; 	// for halflines
TIM_HandleTypeDef *hlcd_tim_delay;		// for us delays
TIM_HandleTypeDef *hlcd_tim_adv;		// for GSP

void lcd_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void lcd_TIM_PWM_PulseFinishedCallback_GSP(TIM_HandleTypeDef *htim);
void lcd_TIM_PWM_PulseFinishedCallback_halfline(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef lcd_TIM_init(TIM_HandleTypeDef *htim_halfline, TIM_HandleTypeDef *htim_delay, TIM_HandleTypeDef *htim_adv ){
	if (htim_halfline == NULL || htim_delay == NULL || htim_adv == NULL)
		return HAL_ERROR;

	hlcd_tim_halfline = htim_halfline;
	hlcd_tim_delay = htim_delay;
	hlcd_tim_adv = htim_adv;

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


void lcd_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if( htim == &htim6 ) {
		static uint8_t rem_matches = 2;

		if( rem_matches == 0 ) {
			HAL_TIM_Base_Stop_IT(&htim6);
			rem_matches = 2;	// reset the counter

			htim15.Instance->CCER |= TIM_CCER_CC1E_Msk;		// (1) enable CC1
		  htim15.Instance->CCER |= TIM_CCER_CC2E_Msk;		// (1) enable CC2
		  htim15.Instance->BDTR |= (TIM_BDTR_MOE);			// (2) master output enable


			HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
//		    TIM1 CH3 is used as TRGO source, triggering TIM15
//		    CH3's interrupt is executed even if PWM is not started
		}
		if( rem_matches == 1 ) {
			// Set GSP high
			HAL_GPIO_WritePin( LCD_GSP_GPIO_Port, LCD_GSP_Pin, GPIO_PIN_SET );
		}
		if( rem_matches )
			rem_matches--;

		HAL_GPIO_TogglePin( test_pin_GPIO_Port, test_pin_Pin );
	}
}

void lcd_TIM_PWM_PulseFinishedCallback_GSP(TIM_HandleTypeDef *htim){
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if( htim == &htim1 ) {
		if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 ) {
			static uint8_t rem_periods = 1;
			if( rem_periods == 0 ) {
				// Set GSP low
				HAL_GPIO_WritePin( LCD_GSP_GPIO_Port, LCD_GSP_Pin, GPIO_PIN_RESET );
				rem_periods = 2;	// reset the counter

				// Disable TIM1 interrupt to avoid redundant ISR calls
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
			}
			if( rem_periods )
				rem_periods--;
		}
	}
}
void lcd_TIM_PWM_PulseFinishedCallback_halfline(TIM_HandleTypeDef *htim){
	if( htim == &htim15 ){
		if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 ) {
			static uint32_t remaining_halflines = 4;

			if( remaining_halflines ) {
				remaining_halflines--;
			}

			// For the first time
			if( remaining_halflines == 3  ) {
	//			HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );

				// Disable TIM15 interrupt not to call it every half-line
				// It will be re-enabled at the last half-line,
				// so the last CC2 match will execute routine
				// that handles disabling some hardware
				__HAL_TIM_DISABLE_IT(&htim15, TIM_IT_CC1);
				HAL_StatusTypeDef ret = HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );	// start TIM1's CH1 (GEN signal)
				if( ret != HAL_OK ){
					sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_PulseFinishedCallback(): HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 ) status = %s\r\n",
							hal_status_str[ret] );
					Error_Handler();
				}

			}
			// Next times
			else {
//				HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );

				if( remaining_halflines == 0 ) {
					__HAL_TIM_DISABLE_IT(&htim15, TIM_IT_CC1);
//					HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
					remaining_halflines = 4;	// reset the counter

					HAL_GPIO_WritePin( LCD_INTB_GPIO_Port, LCD_INTB_Pin, GPIO_PIN_RESET );

					// Disable frame-transmitting hardware
					HAL_StatusTypeDef ret = HAL_OK;
					ret = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					  if( ret != HAL_OK ){
						 sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) status = %s\r\n",
								hal_status_str[ret] );
						 Error_Handler();
					  }

					  ret = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
					  if( ret != HAL_OK ){
						 sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2) status = %s\r\n",
									hal_status_str[ret] );
						 Error_Handler();
					  }

					  ret = HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
					  if( ret != HAL_OK ){
						 sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3) status = %s\r\n",
									hal_status_str[ret] );
						 Error_Handler();
					  }

					  ret = HAL_TIM_OC_Stop(&htim15, TIM_CHANNEL_2);
						if( ret != HAL_OK ){
						   sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2) status = %s\r\n",
										hal_status_str[ret] );
						   Error_Handler();
						}

					  ret = HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
					  if( ret != HAL_OK ){
						 sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1) status = %s\r\n",
									hal_status_str[ret] );
						 Error_Handler();
					  }

					  htim15.Instance->CNT = 0;
					  htim15.Instance->SR = 0;		// clear interrupt flags causing unwanted callback calls
					//  htim15.Instance->EGR |= TIM_EGR_UG_Msk;	// set the UG bit in the EGR register to force the update generation event once the timer is enabled

					  htim1.Instance->CNT = 0;
					  htim1.Instance->SR = 0;		// clear interrupt flags causing unwanted callback calls

					  __HAL_TIM_ENABLE_IT(&htim15, TIM_IT_CC1);		// enable TIM15 interrupt
				}
			}


		}
	}
}

HAL_StatusTypeDef lcd_PWM_power_enable(void){
	if (HAL_TIM_PWM_Start_IT(&htim16, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}

	if (HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}

	return HAL_OK;
}

HAL_StatusTypeDef lcd_PWM_power_disable(void){
	if (HAL_TIM_PWM_Stop_IT(&htim16, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}

	if (HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}

	return HAL_OK;
}
