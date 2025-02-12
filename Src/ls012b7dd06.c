/*
 * ls012b7dd06_stm32.c
 *
 *  Created on: Feb 6, 2025
 *      Author: szyme
 */

#include "ls012b7dd06.h"
#include <stdio.h>	// for sprintf()
//#include "tim.h"	// temp: for timers
#include "main.h"	// for uart_msg_buf

GPIO_TypeDef *hlcd_intb_port;
uint16_t lcd_intb_pin;

void lcd_init(OSPI_HandleTypeDef *hospi,
		TIM_HandleTypeDef *hhalfline_tim, TIM_HandleTypeDef *hdelay_tim,
		TIM_HandleTypeDef *hadv_tim, TIM_HandleTypeDef *htim_pwr,
		GPIO_TypeDef *hintb_port, uint16_t intb_pin){
#ifndef LCD_USE_CUSTOM_CONFIG
	lcd_GPDMA_init();
#endif
	hlcd_intb_port = hintb_port;
	lcd_intb_pin = intb_pin;

	lcd_OSPI_init(hospi);
	lcd_TIM_init(hhalfline_tim, hdelay_tim, hadv_tim, htim_pwr);
}


//
// This function only triggers transmission of a single video frame.
// The related hardware is stopped in HAL_TIM_PWM_PulseFinishedCallback()
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
void lcd_displayFrame( void ){

	HAL_StatusTypeDef ret = HAL_OK;

	  ret = lcd_OSPI_set_cmd_config();

	  if( ret != HAL_OK ){
	  	  sprintf( uart_msg_buf, "Error: HAL_OSPI_Command() status = %s, OSPI error code = 0x%08lX\r\n",
	  	  			hal_status_str[ret], hlcd_ospi->ErrorCode );
	  	  Error_Handler();
	  }

	  // Transmit the first halfline
	  ret = lcd_OSPI_transmit_halfline(0);

	  if( ret != HAL_OK ){
		  sprintf( uart_msg_buf, "Error: HAL_OSPI_Transmit_DMA status = %s, ospi error code = 0x%08lX\r\n",
		  			hal_status_str[ret], hlcd_ospi->ErrorCode );
		  Error_Handler();
	  }

	  // Prepare timers
	  lcd_TIM_prepare();

	  // Set INTB signal,
	  // it will be reset in a timer callback function
	  HAL_GPIO_WritePin( hlcd_intb_port, lcd_intb_pin, GPIO_PIN_SET );

	  ret = HAL_TIM_Base_Start_IT(hlcd_tim_delay);
	  if( ret != HAL_OK ){
		  sprintf( uart_msg_buf, "Error: HAL_TIM_Base_Start_IT(&htim6) status = %s\r\n",
					hal_status_str[ret] );
		  Error_Handler();
	  }
	  // hlcd_tim_adv is started in hlcd_tim_delay's interrupt
}

void lcd_cls( void ) {
	for( uint32_t i = 0; i < OUT_DATA_BUF_SIZE-1-2; i++)
		lcd_tx_buf_1[i] = 0x00;
}

void lcd_setPixel( int16_t x, int16_t y, lcd_colour_t colour ) {
	// Check coordinates against display bounds
	if( x < 0 || y < 0 || x > RLCD_DISP_W || y > RLCD_DISP_H )
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

	uint16_t target_idx = y*2 * OUT_DATA_BUF_LINE_W + x/2;
	uint8_t byte_value = lcd_tx_buf_1[ target_idx ];

	// If x coordinate is odd
	if( x % 2 == 0 ) {
		if( colour.bits.r_msb )
			byte_value |= (1 << GPIO_BUS_R0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_R0_BIT);
		if( colour.bits.g_msb )
			byte_value |= (1 << GPIO_BUS_G0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_G0_BIT);
		if( colour.bits.b_msb )
			byte_value |= (1 << GPIO_BUS_B0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_B0_BIT);
	}
	// If x coordinate is even
	else {
		if( colour.bits.r_msb )
			byte_value |= (1 << GPIO_BUS_R1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_R1_BIT);
		if( colour.bits.g_msb )
			byte_value |= (1 << GPIO_BUS_G1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_G1_BIT);
		if( colour.bits.b_msb )
			byte_value |= (1 << GPIO_BUS_B1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_B1_BIT);
	}

	lcd_tx_buf_1[ target_idx ] = byte_value;

	// Calculate LSB index of the pixel

	target_idx += OUT_DATA_BUF_LINE_W;

	byte_value = lcd_tx_buf_1[ target_idx ];

	// If x coordinate is odd
	if( x % 2 == 0 ) {
		if( colour.bits.r_lsb )
			byte_value |= (1 << GPIO_BUS_R0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_R0_BIT);
		if( colour.bits.g_lsb )
			byte_value |= (1 << GPIO_BUS_G0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_G0_BIT);
		if( colour.bits.b_lsb )
			byte_value |= (1 << GPIO_BUS_B0_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_B0_BIT);
	}
	// If x coordinate is even
	else {
		if( colour.bits.r_lsb )
			byte_value |= (1 << GPIO_BUS_R1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_R1_BIT);
		if( colour.bits.g_lsb )
			byte_value |= (1 << GPIO_BUS_G1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_G1_BIT);
		if( colour.bits.b_lsb )
			byte_value |= (1 << GPIO_BUS_B1_BIT);
		else
			byte_value &= ~(1 << GPIO_BUS_B1_BIT);
	}

	lcd_tx_buf_1[ target_idx ] = byte_value;
}

//
// TODO:
// - skip dummy pixels (their count for each line
//   is in the display's datasheet)
//
void lcd_setColour( lcd_colour_t colour ) {
	for( uint16_t line_no = 0; line_no < RLCD_DISP_H; line_no++ ) {
		// Fill MSB:

		// Start of the whole MSB of image line data
		uint32_t out_buf_line_start_idx = line_no*2 * OUT_DATA_BUF_LINE_W;
		// End of MSB of image line data
		uint32_t out_buf_line_end_idx = out_buf_line_start_idx + OUT_DATA_BUF_LINE_W - 1;

		// Leave a couple of empty periods of the pixel clock at the end
		for( uint8_t i = 0; i < BCK_TRAILING_DUMMY_EDGE_CNT-1; i++ )
			lcd_tx_buf_1[out_buf_line_end_idx-i] = 0x00;

		// Start of the pixel colour data in MSB of image line data
		uint32_t out_buf_line_rgb_start_idx = out_buf_line_start_idx;
		// End of the pixel colour data in MSB of image line data
		uint32_t out_buf_line_rgb_end_idx = out_buf_line_end_idx - BCK_TRAILING_DUMMY_EDGE_CNT+1;

		for( uint32_t odb_i = out_buf_line_rgb_start_idx; odb_i < out_buf_line_rgb_end_idx; odb_i++ ){
			lcd_tx_buf_1[ odb_i ] = colour.val;
		}

		// Fill LSB:

		out_buf_line_start_idx += OUT_DATA_BUF_LINE_W;
		out_buf_line_end_idx = out_buf_line_start_idx + OUT_DATA_BUF_LINE_W - 1;

		// Leave a couple of empty periods of the pixel clock at the end
		for( uint8_t i = 0; i < BCK_TRAILING_DUMMY_EDGE_CNT-1; i++ )
			lcd_tx_buf_1[out_buf_line_end_idx-i] = 0x00;

		// Start of the pixel colour data in MSB of image line data
		out_buf_line_rgb_start_idx = out_buf_line_start_idx;
		// End of the pixel colour data in MSB of image line data
		out_buf_line_rgb_end_idx = out_buf_line_end_idx - BCK_TRAILING_DUMMY_EDGE_CNT+1;

		for( uint32_t odb_i = out_buf_line_rgb_start_idx; odb_i < out_buf_line_rgb_end_idx; odb_i++ ){
			lcd_tx_buf_1[ odb_i ] = colour.val;
		}
	}
}


void lcd_drawSquare( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, lcd_colour_t colour ) {
	for( uint16_t x = x0; x < x1; x++ ) {
		lcd_setPixel( x, y0, colour );
		lcd_setPixel( x, y1, colour );
	}

	for( uint16_t y = y0; y < y1; y++ ) {
		lcd_setPixel( x0, y, colour );
		lcd_setPixel( x1, y, colour );
	}
}

void lcd_drawHLine( uint16_t x0, uint16_t x1, uint16_t y, lcd_colour_t colour ) {
	for( uint16_t x = x0; x < x1; x++ )
		lcd_setPixel( x, y, colour );
}

void lcd_drawTestFigure( void ) {
	lcd_colour_t colour;

	colour.val = COLOUR_YELLOW;
	lcd_setColour( colour );

	colour.val = COLOUR_RED;
	lcd_drawSquare( 60, 60, 100, 100, colour );

	colour.val = COLOUR_GREEN;
	lcd_drawSquare( 120, 60, 160, 100, colour );


	colour.val = COLOUR_BLUE;
	lcd_drawSquare( 60, 120, 100, 160, colour );

	colour.val = COLOUR_WHITE;
	lcd_drawSquare( 120, 120, 160, 160, colour );
}
