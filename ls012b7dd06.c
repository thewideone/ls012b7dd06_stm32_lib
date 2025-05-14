/*
 * ls012b7dd06.c
 *
 *  Created on: Feb 6, 2025
 *      Author: thewideone
 */

#include "ls012b7dd06.h"
#include <stdio.h>	// for sprintf()
#include "main.h"	// for uart_msg_buf

#include <stdlib.h> // for abs()

// Macro for swapping two int16_t variables,
// used in LCD_drawLine()
// Why do {} while(0)? See https://dev.to/pauljlucas/cc-preprocessor-macros-fh5
#define SWAP_INT16(a, b) do {int16_t t = a; a = b; b = t;} while(0)

lcd_ctx_t lcd_ctx[LCD_INSTANCES_CNT] = { 0 };
uint8_t lcd_active_instance_no = 0;

void LCD_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi);
void LCD_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void LCD_TIM_PWM_PulseFinishedCallback_GSP(TIM_HandleTypeDef *htim);
void LCD_TIM_PWM_PulseFinishedCallback_halfline(TIM_HandleTypeDef *htim);

void LCD_init(uint8_t instance_no, lcd_init_t *init) {
	lcd_ctx[instance_no].hospi = init->hospi;
	lcd_ctx[instance_no].hhalfline_tim = init->hhalfline_tim;
	lcd_ctx[instance_no].hdelay_tim = init->hdelay_tim;
	lcd_ctx[instance_no].hadv_tim = init->hadv_tim;
	lcd_ctx[instance_no].hpwr_tim = init->hpwr_tim;
	lcd_ctx[instance_no].hintb_port = init->hintb_port;

	lcd_ctx[instance_no].intb_pin = init->intb_pin;
	lcd_ctx[instance_no].active = false;
	lcd_ctx[instance_no].busy = false;
	lcd_ctx[instance_no].buf1 = init->buf1;

	HAL_StatusTypeDef ret = HAL_OSPI_RegisterCallback(
			lcd_ctx[instance_no].hospi, HAL_OSPI_TX_CPLT_CB_ID,
			LCD_OSPI_TxCpltCallback);

	// In a single transmission,
	// first, an instruction (1<<BSP_PIN) is transmitted to send
	// the BSP signal pulse
	// ( apparently, the transmission can not lack both
	// instruction and address phases at the same time
	// see body of OSPI_ConfigCmd() used by HAL_OSPI_Command() );
	// next, the data is sent with two trailing 0x00 bytes
	// to add two extra dummy clock edges.

	lcd_ctx[instance_no].ospi_cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	lcd_ctx[instance_no].ospi_cmd.Instruction = (1 << 6);//(1 << LCD_BSP_Pin);//0x20;			// send the BSP signal pulse
	lcd_ctx[instance_no].ospi_cmd.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;	// 8-line parallel output
	lcd_ctx[instance_no].ospi_cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;// transmit the BSP signal on only one edge of the clock
	//		  .Address = 0x00,
	//		  .AddressSize = HAL_OSPI_ADDRESS_8_BITS,
	lcd_ctx[instance_no].ospi_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;	// no address phase
	lcd_ctx[instance_no].ospi_cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	lcd_ctx[instance_no].ospi_cmd.DataMode = HAL_OSPI_DATA_8_LINES;	// 8-line parallel output
	lcd_ctx[instance_no].ospi_cmd.NbData = sizeof(uint8_t) * OUT_DATA_BUF_LINE_W; // size of data to be transmitted, possible range: 1 - 0xFFFFFFFF (1-2^32)
	lcd_ctx[instance_no].ospi_cmd.DataDtrMode = HAL_OSPI_DATA_DTR_ENABLE; // transmit on both edges of the clock
	lcd_ctx[instance_no].ospi_cmd.DummyCycles = 0; // 0-31
	lcd_ctx[instance_no].ospi_cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
	lcd_ctx[instance_no].ospi_cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD; // send BSP on every transmission

	if (init->hospi == NULL || init->hhalfline_tim == NULL
			|| init->hdelay_tim == NULL || init->hadv_tim == NULL
			|| init->hpwr_tim == NULL) {
		sprintf(uart_msg_buf,
				"Error: lcd_init(): at least one handle is null\r\n");
//		return HAL_ERROR;
		Error_Handler();
	}

	ret = HAL_TIM_RegisterCallback(lcd_ctx[instance_no].hdelay_tim,
			HAL_TIM_PERIOD_ELAPSED_CB_ID, LCD_TIM_PeriodElapsedCallback);
	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_init(): HAL_TIM_RegisterCallback(hdelay_tim) status = %s\r\n",
				hal_status_str[ret]);
		Error_Handler();
	}

	ret = HAL_TIM_RegisterCallback(lcd_ctx[instance_no].hadv_tim,
			HAL_TIM_PWM_PULSE_FINISHED_CB_ID,
			LCD_TIM_PWM_PulseFinishedCallback_GSP);
	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_init(): HAL_TIM_RegisterCallback(hadv_tim) status = %s\r\n",
				hal_status_str[ret]);
		Error_Handler();
	}

	ret = HAL_TIM_RegisterCallback(lcd_ctx[instance_no].hhalfline_tim,
			HAL_TIM_PWM_PULSE_FINISHED_CB_ID,
			LCD_TIM_PWM_PulseFinishedCallback_halfline);
	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_init(): HAL_TIM_RegisterCallback(hhalfline_tim) status = %s\r\n",
				hal_status_str[ret]);
		Error_Handler();
	}
}

void LCD_setActive(uint8_t instance_no) {
	// TODO: check if possible
	// and stop currently active instance if necessary

	lcd_active_instance_no = instance_no;
	lcd_ctx[lcd_active_instance_no].active = true;
}

//void LCD_setInactive(uint8_t instance_no){
// TODO: check if possible
// and stop currently active instance if necessary
//}

//
// OSPI functions
//

// Used also in display_frame()
HAL_StatusTypeDef LCD_OSPI_set_cmd_config(void) {
	return HAL_OSPI_Command(lcd_ctx[lcd_active_instance_no].hospi,
			&(lcd_ctx[lcd_active_instance_no].ospi_cmd), 2047);
}

// Used also in display_frame()
HAL_StatusTypeDef LCD_OSPI_transmit_halfline(uint32_t halfline_no) {
	return HAL_OSPI_Transmit_DMA(lcd_ctx[lcd_active_instance_no].hospi,
			(uint8_t*) lcd_ctx[lcd_active_instance_no].buf1
					+ sizeof(lcd_colour_t) * OUT_DATA_BUF_LINE_W * halfline_no);
}

// callbacks can be executed for multiple e.g. timers
// use user callbacks instead!
//void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi){
void LCD_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi) {
//	HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
//	sprintf( uart_msg_buf, "OSPI1 tx complete. lcd_ctx[lcd_active_instance_no].hospi->State = 0x%08X. Transmitting next...\r\n",
//						lcd_ctx[lcd_active_instance_no].hospi.State );
//	HAL_UART_Transmit( &huart1, uart_msg_buf, sizeof(uart_msg_buf), 10 );

	HAL_StatusTypeDef ret = HAL_OK;

	// Initially 1 because this callback is executed after half-line has already been transmitted
	static uint32_t halfline_no = 1;

	if (halfline_no < 480) {
//		HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
		ret = LCD_OSPI_set_cmd_config();

		if (ret != HAL_OK) {
			sprintf(uart_msg_buf,
					"Error: lcd_OSPI_TxCpltCallback(): lcd_OSPI_set_cmd_config() status = %s, OSPI error code = 0x%08lX\r\n",
					hal_status_str[ret], hospi->ErrorCode);
			Error_Handler();
		}

		// Transmit next halfline
//		ret = HAL_OSPI_Transmit_DMA( hospi, (uint8_t*)lcd_tx_buf_1 + sizeof(lcd_colour_t)*OUT_DATA_BUF_LINE_W*halfline_no );
		ret = LCD_OSPI_transmit_halfline(halfline_no);

		if (ret != HAL_OK) {
			sprintf(uart_msg_buf,
					"Error: lcd_OSPI_TxCpltCallback(): lcd_OSPI_transmit_halfline(halfline_no) status = %s, ospi error code = 0x%08lX\r\n",
					hal_status_str[ret], hospi->ErrorCode);
			Error_Handler();
		}
//		HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );

		halfline_no++;
	} else {
		halfline_no = 1;
//		HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
		// Clear TIM15 CC1 IF
		__HAL_TIM_CLEAR_FLAG(lcd_ctx[lcd_active_instance_no].hhalfline_tim, TIM_FLAG_CC1);
		__HAL_TIM_ENABLE_IT(lcd_ctx[lcd_active_instance_no].hhalfline_tim, TIM_IT_CC1);
//		HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
	}

//	HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
}

//__weak void lcd_OSPI_Error_Handler(){
//
//}

//
// Timers functions
//

void LCD_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == lcd_ctx[lcd_active_instance_no].hdelay_tim) {
		static uint8_t rem_matches = 2;

		if (rem_matches == 0) {
			HAL_TIM_Base_Stop_IT(lcd_ctx[lcd_active_instance_no].hdelay_tim);
			rem_matches = 2;	// reset the counter

			lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->CCER |= TIM_CCER_CC1E_Msk;		// (1) enable CC1
			lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->CCER |= TIM_CCER_CC2E_Msk;		// (1) enable CC2
			lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->BDTR |= (TIM_BDTR_MOE);		// (2) master output enable

			HAL_TIM_PWM_Start(lcd_ctx[lcd_active_instance_no].hadv_tim, TIM_CHANNEL_2);
//		    TIM1 CH3 is used as TRGO source, triggering TIM15
//		    CH3's interrupt is executed even if PWM is not started
		}
		if (rem_matches == 1) {
			// Set GSP high
			HAL_GPIO_WritePin( LCD_GSP_GPIO_Port, LCD_GSP_Pin, GPIO_PIN_SET);
		}
		if (rem_matches)
			rem_matches--;

		HAL_GPIO_TogglePin( test_pin_GPIO_Port, test_pin_Pin);
	}
}

void LCD_TIM_PWM_PulseFinishedCallback_GSP(TIM_HandleTypeDef *htim) {
	if (htim == lcd_ctx[lcd_active_instance_no].hadv_tim) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			static uint8_t rem_periods = 1;
			if (rem_periods == 0) {
				// Set GSP low
				HAL_GPIO_WritePin( LCD_GSP_GPIO_Port, LCD_GSP_Pin, GPIO_PIN_RESET);
				rem_periods = 2;	// reset the counter

				// Disable TIM1 interrupt to avoid redundant ISR calls
				__HAL_TIM_DISABLE_IT(lcd_ctx[lcd_active_instance_no].hadv_tim, TIM_IT_CC1);
			}
			if (rem_periods)
				rem_periods--;
		}
	}
}
void LCD_TIM_PWM_PulseFinishedCallback_halfline(TIM_HandleTypeDef *htim) {
	if (htim == lcd_ctx[lcd_active_instance_no].hhalfline_tim) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			static uint32_t remaining_halflines = 4;

			if (remaining_halflines) {
				remaining_halflines--;
			}

			// For the first time
			if (remaining_halflines == 3) {
				//			HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );

				// Disable TIM15 interrupt not to call it every half-line
				// It will be re-enabled at the last half-line,
				// so the last CC2 match will execute routine
				// that handles disabling some hardware
				__HAL_TIM_DISABLE_IT(
						lcd_ctx[lcd_active_instance_no].hhalfline_tim,
						TIM_IT_CC1);
				HAL_StatusTypeDef ret = HAL_TIM_PWM_Start(
						lcd_ctx[lcd_active_instance_no].hadv_tim,
						TIM_CHANNEL_1);	// start TIM1's CH1 (GEN signal)
				if (ret != HAL_OK) {
					sprintf(uart_msg_buf,
							"Error: HAL_TIM_PWM_PulseFinishedCallback(): HAL_TIM_PWM_Start( lcd_ctx[lcd_active_instance_no].hadv_tim, TIM_CHANNEL_1 ) status = %s\r\n",
							hal_status_str[ret]);
					Error_Handler();
				}

			}
			// Next times
			else {
//				HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );

				if (remaining_halflines == 0) {
					__HAL_TIM_DISABLE_IT(
							lcd_ctx[lcd_active_instance_no].hhalfline_tim,
							TIM_IT_CC1);
//					HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_6 );
					remaining_halflines = 4;	// reset the counter

					HAL_GPIO_WritePin( LCD_INTB_GPIO_Port, LCD_INTB_Pin,
							GPIO_PIN_RESET);

					// End of frame transmission,
					// disable the frame-transmitting hardware
					HAL_StatusTypeDef ret = HAL_OK;
					ret = HAL_TIM_PWM_Stop(
							lcd_ctx[lcd_active_instance_no].hadv_tim,
							TIM_CHANNEL_1);
					if (ret != HAL_OK) {
						sprintf(uart_msg_buf,
								"Error: HAL_TIM_PWM_Stop(lcd_ctx[lcd_active_instance_no].hadv_tim, TIM_CHANNEL_1) status = %s\r\n",
								hal_status_str[ret]);
						Error_Handler();
					}

					ret = HAL_TIM_PWM_Stop(
							lcd_ctx[lcd_active_instance_no].hadv_tim,
							TIM_CHANNEL_2);
					if (ret != HAL_OK) {
						sprintf(uart_msg_buf,
								"Error: HAL_TIM_PWM_Stop(lcd_ctx[lcd_active_instance_no].hadv_tim, TIM_CHANNEL_2) status = %s\r\n",
								hal_status_str[ret]);
						Error_Handler();
					}

					ret = HAL_TIM_PWM_Stop(
							lcd_ctx[lcd_active_instance_no].hadv_tim,
							TIM_CHANNEL_3);
					if (ret != HAL_OK) {
						sprintf(uart_msg_buf,
								"Error: HAL_TIM_PWM_Stop(lcd_ctx[lcd_active_instance_no].hadv_tim, TIM_CHANNEL_3) status = %s\r\n",
								hal_status_str[ret]);
						Error_Handler();
					}

					ret = HAL_TIM_OC_Stop(
							lcd_ctx[lcd_active_instance_no].hhalfline_tim,
							TIM_CHANNEL_2);
					if (ret != HAL_OK) {
						sprintf(uart_msg_buf,
								"Error: HAL_TIM_PWM_Stop(lcd_ctx[lcd_active_instance_no].hhalfline_tim, TIM_CHANNEL_2) status = %s\r\n",
								hal_status_str[ret]);
						Error_Handler();
					}

					ret = HAL_TIM_PWM_Stop(
							lcd_ctx[lcd_active_instance_no].hhalfline_tim,
							TIM_CHANNEL_1);
					if (ret != HAL_OK) {
						sprintf(uart_msg_buf,
								"Error: HAL_TIM_PWM_Stop(lcd_ctx[lcd_active_instance_no].hhalfline_tim, TIM_CHANNEL_1) status = %s\r\n",
								hal_status_str[ret]);
						Error_Handler();
					}

					lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->CNT = 0;
					lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->SR = 0;// clear interrupt flags causing unwanted callback calls
					//  htim15.Instance->EGR |= TIM_EGR_UG_Msk;	// set the UG bit in the EGR register to force the update generation event once the timer is enabled

					lcd_ctx[lcd_active_instance_no].hadv_tim->Instance->CNT = 0;
					lcd_ctx[lcd_active_instance_no].hadv_tim->Instance->SR = 0;	// clear interrupt flags causing unwanted callback calls

					__HAL_TIM_ENABLE_IT(
							lcd_ctx[lcd_active_instance_no].hhalfline_tim,
							TIM_IT_CC1);		// enable TIM15 interrupt

					lcd_ctx[lcd_active_instance_no].busy = false;
				}
			}

		}
	}
}

// Start VA, and VB and VCOM signals
HAL_StatusTypeDef LCD_PWM_power_enable(void) {
	HAL_StatusTypeDef ret;

	ret = HAL_TIM_PWM_Start_IT(lcd_ctx[lcd_active_instance_no].hpwr_tim, TIM_CHANNEL_1);
	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_PWM_power_enable(): HAL_TIM_PWM_Start_IT(hpwr_tim) status = %s\r\n",
				hal_status_str[ret]);
		Error_Handler();
	}

	ret = HAL_TIMEx_PWMN_Start(lcd_ctx[lcd_active_instance_no].hpwr_tim, TIM_CHANNEL_1);
	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_PWM_power_enable(): HAL_TIMEx_PWMN_Start(hpwr_tim) status = %s\r\n",
				hal_status_str[ret]);
		Error_Handler();
	}

	return HAL_OK;
}

// Stop VA, and VB and VCOM signals
HAL_StatusTypeDef LCD_PWM_power_disable(void) {
	if (lcd_ctx[lcd_active_instance_no].busy)
		return HAL_BUSY;

	HAL_StatusTypeDef ret;

	ret = HAL_TIM_PWM_Stop_IT(lcd_ctx[lcd_active_instance_no].hpwr_tim, TIM_CHANNEL_1);
	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_PWM_power_disable(): HAL_TIM_PWM_Stop_IT(hpwr_tim) status = %s\r\n",
				hal_status_str[ret]);
		Error_Handler();
	}

	ret = HAL_TIMEx_PWMN_Stop(lcd_ctx[lcd_active_instance_no].hpwr_tim, TIM_CHANNEL_1);
	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_PWM_power_disable(): HAL_TIM_PWM_Stop_IT(hpwr_tim) status = %s\r\n",
				hal_status_str[ret]);
		Error_Handler();
	}

	return HAL_OK;
}

void LCD_TIM_prepare(void) {
	lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->CNT = 0;// reset counter
	lcd_ctx[lcd_active_instance_no].hadv_tim->Instance->CNT = 0;// reset counter
	lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->SR = 0;// clear interrupt flags causing unwanted callback calls
	lcd_ctx[lcd_active_instance_no].hadv_tim->Instance->SR = 0;	// clear interrupt flags causing unwanted callback calls

	__HAL_TIM_ENABLE_IT(lcd_ctx[lcd_active_instance_no].hhalfline_tim, TIM_IT_CC1);		// enable TIM15 interrupt
	__HAL_TIM_ENABLE_IT(lcd_ctx[lcd_active_instance_no].hadv_tim, TIM_IT_CC1);// enable TIM15 interrupt

	lcd_ctx[lcd_active_instance_no].hhalfline_tim->ChannelState[0] =
			(HAL_TIM_CHANNEL_STATE_BUSY);
	lcd_ctx[lcd_active_instance_no].hhalfline_tim->ChannelState[1] =
			(HAL_TIM_CHANNEL_STATE_BUSY);

	// Reset timer's output compare state in one-pulse mode
	// Source:
	// https://community.st.com/t5/stm32-mcus-products/reset-timer-output-compare-output-with-timer-in-one-pulse-mode/td-p/219631
	uint32_t tmp;

	tmp = lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->CCMR1;

	tmp &= ~TIM_CCMR1_OC2M_0;
	tmp &= ~TIM_CCMR1_OC2M_1;
	tmp |= TIM_CCMR1_OC2M_2;
	tmp &= ~TIM_CCMR1_OC2M_3;

	lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->CCMR1 = tmp;

	tmp |= TIM_CCMR1_OC2M_0;
	tmp |= TIM_CCMR1_OC2M_1;
	tmp &= ~TIM_CCMR1_OC2M_2;
	tmp &= ~TIM_CCMR1_OC2M_3;

	lcd_ctx[lcd_active_instance_no].hhalfline_tim->Instance->CCMR1 = tmp;

	// Enable interrupt on lcd_ctx[lcd_active_instance_no].hadv_tim.
	// It is being disabled
	// in lcd_TIM_PWM_PulseFinishedCallback_GSP()
	// to avoid redundant ISR calls
	__HAL_TIM_ENABLE_IT(lcd_ctx[lcd_active_instance_no].hadv_tim, TIM_IT_CC1);
}

//
// General functions
//

//
// This function only triggers transmission of a single video frame.
// The related hardware is stopped in LCD_TIM_PWM_PulseFinishedCallback_halfline()
// once the frame transmission ends.
//
// TIM6 is used as a us delay generator:
// - 10us between rising edges of INTB, GSP, and GCK
// - 30us between end of the initialisation and
//   the first rising edge of the VA signal
// - 30us between last VA falling edge and
//   possible VDD2 falling edge (LCD's 5V supply shutdown)
//
// TIM1's CH3 is a trigger source for TIM15.
//
// Pixel data transmitted by OSPI. Transmission triggered by TIM15 CH1 PWM.
// BSP as data line in OSPI.
// GCK - TIM15 CH2 toggle on compare match.
// GEN - TIM1 CH1 + CH2 combined, TIM1's CH1 output started by
// 		 TIM15 interrupt in HAL_TIM_PWM_PulseFinishedCallback()
//
// Initial empty GCK high state is forced by CCMR1 register of TIM15.
// Width of this state is controlled by TIM1's CH3 pulse value.
//
HAL_StatusTypeDef LCD_displayFrame(void) {
	if (lcd_ctx[lcd_active_instance_no].busy)
		return HAL_BUSY;

	lcd_ctx[lcd_active_instance_no].busy = true;

	HAL_StatusTypeDef ret = HAL_OK;

	ret = LCD_OSPI_set_cmd_config();

	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_displayFrame(): lcd_OSPI_set_cmd_config() status = %s, OSPI error code = 0x%08lX\r\n",
				hal_status_str[ret],
				lcd_ctx[lcd_active_instance_no].hospi->ErrorCode);
		Error_Handler();
	}

	// Transmit the first halfline
	ret = LCD_OSPI_transmit_halfline(0);

	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_displayFrame(): lcd_OSPI_transmit_halfline(0) status = %s, ospi error code = 0x%08lX\r\n",
				hal_status_str[ret],
				lcd_ctx[lcd_active_instance_no].hospi->ErrorCode);
		Error_Handler();
	}

	// Prepare timers
	LCD_TIM_prepare();

	// Set INTB signal,
	// it will be reset in a timer callback function
	HAL_GPIO_WritePin(lcd_ctx[lcd_active_instance_no].hintb_port,
			lcd_ctx[lcd_active_instance_no].intb_pin, GPIO_PIN_SET);

	ret = HAL_TIM_Base_Start_IT(lcd_ctx[lcd_active_instance_no].hdelay_tim);
	if (ret != HAL_OK) {
		sprintf(uart_msg_buf,
				"Error: lcd_displayFrame(): HAL_TIM_Base_Start_IT(&htim6) status = %s\r\n",
				hal_status_str[ret]);
		Error_Handler();
	}
	// lcd_ctx[lcd_active_instance_no].hadv_tim is started in lcd_ctx[lcd_active_instance_no].hdelay_tim's interrupt

	return HAL_OK;
}

void LCD_cls(void) {
	for (uint32_t i = 0; i < OUT_DATA_BUF_SIZE - 1 - 2; i++)
		lcd_ctx[lcd_active_instance_no].buf1[i] = 0x00;
}

void LCD_setPixel(int16_t x, int16_t y, lcd_colour_t colour) {
	// Check coordinates against display bounds
	if (x < 0 || y < 0 || x > RLCD_DISP_W || y > RLCD_DISP_H)
		return;

	// Calculate MSB index of the pixel

	// y == vertical line no.
	// y*2 == MSB half-line no. (MSB is transmitted first)
	// y*2 + 1 == LSB half-line no.
	//
	// x%2 == 0 => this is an odd pixel => apply the colour to pin R[0]
	// else, to R[1] pin
	// Because in the datasheet of the display, pixels are numbered from 1
	// and indices in C from 0, an odd pixel in datasheet's numbering is even in C code.
	//
	// horizontal half-line offset = floor(x/2) = x/2 (by definition in C)

	uint16_t target_idx = y * 2 * OUT_DATA_BUF_LINE_W + x / 2;
	uint8_t byte_value = lcd_ctx[lcd_active_instance_no].buf1[target_idx];

	// If x coordinate is odd
	if (x % 2 == 0) {
		if (colour.bits.r_msb)
			byte_value |= (1 << GPIO_BUS_R0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_R0_BIT);
		if (colour.bits.g_msb)
			byte_value |= (1 << GPIO_BUS_G0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_G0_BIT);
		if (colour.bits.b_msb)
			byte_value |= (1 << GPIO_BUS_B0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_B0_BIT);
	}
	// If x coordinate is even
	else {
		if (colour.bits.r_msb)
			byte_value |= (1 << GPIO_BUS_R1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_R1_BIT);
		if (colour.bits.g_msb)
			byte_value |= (1 << GPIO_BUS_G1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_G1_BIT);
		if (colour.bits.b_msb)
			byte_value |= (1 << GPIO_BUS_B1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_B1_BIT);
	}

	lcd_ctx[lcd_active_instance_no].buf1[target_idx] = byte_value;

	// Calculate LSB index of the pixel

	target_idx += OUT_DATA_BUF_LINE_W;

	byte_value = lcd_ctx[lcd_active_instance_no].buf1[target_idx];

	// If x coordinate is odd
	if (x % 2 == 0) {
		if (colour.bits.r_lsb)
			byte_value |= (1 << GPIO_BUS_R0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_R0_BIT);
		if (colour.bits.g_lsb)
			byte_value |= (1 << GPIO_BUS_G0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_G0_BIT);
		if (colour.bits.b_lsb)
			byte_value |= (1 << GPIO_BUS_B0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_B0_BIT);
	}
	// If x coordinate is even
	else {
		if (colour.bits.r_lsb)
			byte_value |= (1 << GPIO_BUS_R1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_R1_BIT);
		if (colour.bits.g_lsb)
			byte_value |= (1 << GPIO_BUS_G1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_G1_BIT);
		if (colour.bits.b_lsb)
			byte_value |= (1 << GPIO_BUS_B1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_B1_BIT);
	}

	lcd_ctx[lcd_active_instance_no].buf1[target_idx] = byte_value;
}

void LCD_setColour(lcd_colour_t colour) {
	for (uint16_t line_no = 0; line_no < RLCD_DISP_H; line_no++) {
		// Fill MSB:

		// Start of the whole MSB of image line data
		uint32_t out_buf_line_start_idx = line_no * 2 * OUT_DATA_BUF_LINE_W;
		// End of MSB of image line data
		uint32_t out_buf_line_end_idx = out_buf_line_start_idx
				+ OUT_DATA_BUF_LINE_W - 1;

		// Leave a couple of empty periods of the pixel clock at the end
		for (uint8_t i = 0; i < BCK_TRAILING_DUMMY_EDGE_CNT - 1; i++)
			lcd_ctx[lcd_active_instance_no].buf1[out_buf_line_end_idx - i] = 0x00;

		// Start of the pixel colour data in MSB of image line data
		uint32_t out_buf_line_rgb_start_idx = out_buf_line_start_idx;
		// End of the pixel colour data in MSB of image line data
		uint32_t out_buf_line_rgb_end_idx = out_buf_line_end_idx
				- BCK_TRAILING_DUMMY_EDGE_CNT + 1;

		for (uint32_t odb_i = out_buf_line_rgb_start_idx;
				odb_i < out_buf_line_rgb_end_idx; odb_i++) {
			lcd_ctx[lcd_active_instance_no].buf1[odb_i] = colour.val;
		}

		// Fill LSB:

		out_buf_line_start_idx += OUT_DATA_BUF_LINE_W;
		out_buf_line_end_idx = out_buf_line_start_idx + OUT_DATA_BUF_LINE_W - 1;

		// Leave a couple of empty periods of the pixel clock at the end
		for (uint8_t i = 0; i < BCK_TRAILING_DUMMY_EDGE_CNT - 1; i++)
			lcd_ctx[lcd_active_instance_no].buf1[out_buf_line_end_idx - i] = 0x00;

		// Start of the pixel colour data in MSB of image line data
		out_buf_line_rgb_start_idx = out_buf_line_start_idx;
		// End of the pixel colour data in MSB of image line data
		out_buf_line_rgb_end_idx = out_buf_line_end_idx
				- BCK_TRAILING_DUMMY_EDGE_CNT + 1;

		for (uint32_t odb_i = out_buf_line_rgb_start_idx;
				odb_i < out_buf_line_rgb_end_idx; odb_i++) {
			lcd_ctx[lcd_active_instance_no].buf1[odb_i] = colour.val;
		}
	}
}

void LCD_drawSquare(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		lcd_colour_t colour) {
	for (uint16_t x = x0; x < x1; x++) {
		LCD_setPixel(x, y0, colour);
		LCD_setPixel(x, y1, colour);
	}

	for (uint16_t y = y0; y < y1; y++) {
		LCD_setPixel(x0, y, colour);
		LCD_setPixel(x1, y, colour);
	}
}

void LCD_drawHLine(uint16_t x0, uint16_t x1, uint16_t y, lcd_colour_t colour) {
	for (uint16_t x = x0; x < x1; x++)
		LCD_setPixel(x, y, colour);
}

// Bresenham's algorithm
void LCD_drawLine( int16_t x0, int16_t y0, int16_t x1, int16_t y1, lcd_colour_t color ){

	int steep = abs( y1 - y0 ) > abs( x1 - x0 );

	if (steep) {
		SWAP_INT16( x0, y0 );
		SWAP_INT16( x1, y1 );
	}

	if ( x0 > x1 ) {
		SWAP_INT16( x0, x1 );
		SWAP_INT16( y0, y1 );
	}

	int dx, dy;
	dx = x1 - x0;
	dy = abs( y1 - y0 );

	int err = dx / 2;
	int ystep;

	if ( y0 < y1 )
		ystep = 1;
	else
		ystep = -1;

	for ( ; x0 <= x1; x0++ ) {
		if(steep)
			LCD_setPixel( y0, x0, color );
		else
			LCD_setPixel( x0, y0, color );

		err -= dy;

		if ( err < 0 ) {
			y0 += ystep;
			err += dx;
		}
	}
}

void LCD_drawTestFigure(void) {
	lcd_colour_t colour;

	colour.val = COLOUR_WHITE;
	LCD_setColour(colour);

	colour.val = COLOUR_RED;
	LCD_drawSquare(60, 60, 100, 100, colour);

	colour.val = COLOUR_GREEN;
	LCD_drawSquare(120, 60, 160, 100, colour);

	colour.val = COLOUR_BLUE;
	LCD_drawSquare(60, 120, 100, 160, colour);

	colour.val = COLOUR_YELLOW;
	LCD_drawSquare(120, 120, 160, 160, colour);
}
