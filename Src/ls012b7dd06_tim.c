/*
 * ls012b7dd06_tim.c
 *
 *  Created on: Feb 11, 2025
 *      Author: szyme
 */

#include "ls012b7dd06_tim.h"
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


void lcd_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if( htim == hlcd_tim_delay ) {
		static uint8_t rem_matches = 2;

		if( rem_matches == 0 ) {
			HAL_TIM_Base_Stop_IT(hlcd_tim_delay);
			rem_matches = 2;	// reset the counter

			htim15.Instance->CCER |= TIM_CCER_CC1E_Msk;		// (1) enable CC1
		  htim15.Instance->CCER |= TIM_CCER_CC2E_Msk;		// (1) enable CC2
		  htim15.Instance->BDTR |= (TIM_BDTR_MOE);			// (2) master output enable


			HAL_TIM_PWM_Start( hlcd_tim_adv, TIM_CHANNEL_2 );
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
	if( htim == hlcd_tim_adv ) {
		if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 ) {
			static uint8_t rem_periods = 1;
			if( rem_periods == 0 ) {
				// Set GSP low
				HAL_GPIO_WritePin( LCD_GSP_GPIO_Port, LCD_GSP_Pin, GPIO_PIN_RESET );
				rem_periods = 2;	// reset the counter

				// Disable TIM1 interrupt to avoid redundant ISR calls
				__HAL_TIM_DISABLE_IT(hlcd_tim_adv, TIM_IT_CC1);
			}
			if( rem_periods )
				rem_periods--;
		}
	}
}
void lcd_TIM_PWM_PulseFinishedCallback_halfline(TIM_HandleTypeDef *htim){
	if( htim == hlcd_tim_halfline ){
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
				__HAL_TIM_DISABLE_IT(hlcd_tim_halfline, TIM_IT_CC1);
				HAL_StatusTypeDef ret = HAL_TIM_PWM_Start( hlcd_tim_adv, TIM_CHANNEL_1 );	// start TIM1's CH1 (GEN signal)
				if( ret != HAL_OK ){
					sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_PulseFinishedCallback(): HAL_TIM_PWM_Start( hlcd_tim_adv, TIM_CHANNEL_1 ) status = %s\r\n",
							hal_status_str[ret] );
					Error_Handler();
				}

			}
			// Next times
			else {
//				HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );

				if( remaining_halflines == 0 ) {
					__HAL_TIM_DISABLE_IT(hlcd_tim_halfline, TIM_IT_CC1);
//					HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
					remaining_halflines = 4;	// reset the counter

					HAL_GPIO_WritePin( LCD_INTB_GPIO_Port, LCD_INTB_Pin, GPIO_PIN_RESET );

					// Disable frame-transmitting hardware
					HAL_StatusTypeDef ret = HAL_OK;
					ret = HAL_TIM_PWM_Stop(hlcd_tim_adv, TIM_CHANNEL_1);
					  if( ret != HAL_OK ){
						 sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(hlcd_tim_adv, TIM_CHANNEL_1) status = %s\r\n",
								hal_status_str[ret] );
						 Error_Handler();
					  }

					  ret = HAL_TIM_PWM_Stop(hlcd_tim_adv, TIM_CHANNEL_2);
					  if( ret != HAL_OK ){
						 sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(hlcd_tim_adv, TIM_CHANNEL_2) status = %s\r\n",
									hal_status_str[ret] );
						 Error_Handler();
					  }

					  ret = HAL_TIM_PWM_Stop(hlcd_tim_adv, TIM_CHANNEL_3);
					  if( ret != HAL_OK ){
						 sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(hlcd_tim_adv, TIM_CHANNEL_3) status = %s\r\n",
									hal_status_str[ret] );
						 Error_Handler();
					  }

					  ret = HAL_TIM_OC_Stop(hlcd_tim_halfline, TIM_CHANNEL_2);
						if( ret != HAL_OK ){
						   sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(hlcd_tim_halfline, TIM_CHANNEL_2) status = %s\r\n",
										hal_status_str[ret] );
						   Error_Handler();
						}

					  ret = HAL_TIM_PWM_Stop(hlcd_tim_halfline, TIM_CHANNEL_1);
					  if( ret != HAL_OK ){
						 sprintf( uart_msg_buf, "Error: HAL_TIM_PWM_Stop(hlcd_tim_halfline, TIM_CHANNEL_1) status = %s\r\n",
									hal_status_str[ret] );
						 Error_Handler();
					  }

					  htim15.Instance->CNT = 0;
					  htim15.Instance->SR = 0;		// clear interrupt flags causing unwanted callback calls
					//  htim15.Instance->EGR |= TIM_EGR_UG_Msk;	// set the UG bit in the EGR register to force the update generation event once the timer is enabled

					  htim1.Instance->CNT = 0;
					  htim1.Instance->SR = 0;		// clear interrupt flags causing unwanted callback calls

					  __HAL_TIM_ENABLE_IT(hlcd_tim_halfline, TIM_IT_CC1);		// enable TIM15 interrupt
				}
			}


		}
	}
}

// Start VA, and VB and VCOM signals
HAL_StatusTypeDef lcd_PWM_power_enable(void){
	if (HAL_TIM_PWM_Start_IT(hlcd_tim_pwr, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}

	if (HAL_TIMEx_PWMN_Start(hlcd_tim_pwr, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}

	return HAL_OK;
}

// Stop VA, and VB and VCOM signals
HAL_StatusTypeDef lcd_PWM_power_disable(void){
	if (HAL_TIM_PWM_Stop_IT(hlcd_tim_pwr, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}

	if (HAL_TIMEx_PWMN_Stop(hlcd_tim_pwr, TIM_CHANNEL_1) != HAL_OK){
		Error_Handler();
	}

	return HAL_OK;
}

void lcd_TIM_prepare(void) {
	hlcd_tim_halfline->Instance->CNT = 0;	// reset counter
	hlcd_tim_adv->Instance->CNT = 0;		// reset counter
	hlcd_tim_halfline->Instance->SR = 0;	// clear interrupt flags causing unwanted callback calls
	hlcd_tim_adv->Instance->SR = 0;			// clear interrupt flags causing unwanted callback calls

	__HAL_TIM_ENABLE_IT(hlcd_tim_halfline, TIM_IT_CC1);		// enable TIM15 interrupt
	__HAL_TIM_ENABLE_IT(hlcd_tim_adv, TIM_IT_CC1);			// enable TIM15 interrupt

	  hlcd_tim_halfline->ChannelState[0] = (HAL_TIM_CHANNEL_STATE_BUSY);
	  hlcd_tim_halfline->ChannelState[1] = (HAL_TIM_CHANNEL_STATE_BUSY);

	  // Reset timer's output compare state in one-pulse mode
	  // Source:
	  // https://community.st.com/t5/stm32-mcus-products/reset-timer-output-compare-output-with-timer-in-one-pulse-mode/td-p/219631
	  uint32_t tmp;

	  tmp = hlcd_tim_halfline->Instance->CCMR1;

	  tmp &= ~TIM_CCMR1_OC2M_0;
	  tmp &= ~TIM_CCMR1_OC2M_1;
	  tmp |= TIM_CCMR1_OC2M_2;
	  tmp &= ~TIM_CCMR1_OC2M_3;

	  hlcd_tim_halfline->Instance->CCMR1 = tmp;

	  tmp |= TIM_CCMR1_OC2M_0;
	  tmp |= TIM_CCMR1_OC2M_1;
	  tmp &= ~TIM_CCMR1_OC2M_2;
	  tmp &= ~TIM_CCMR1_OC2M_3;

	  hlcd_tim_halfline->Instance->CCMR1 = tmp;

	  // Enable interrupt on hlcd_tim_adv.
	  // It is being disabled
	  // in lcd_TIM_PWM_PulseFinishedCallback_GSP()
	  // to avoid redundant ISR calls
	  __HAL_TIM_ENABLE_IT(hlcd_tim_adv, TIM_IT_CC1);
}
