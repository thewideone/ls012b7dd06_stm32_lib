/*
 * ls012b7dd06_stm32.c
 *
 *  Created on: Feb 6, 2025
 *      Author: szyme
 */

#include "ls012b7dd06.h"

void lcd_init(OSPI_HandleTypeDef *hospi, TIM_HandleTypeDef *hhalfline_tim, TIM_HandleTypeDef *hdelay_tim, TIM_HandleTypeDef *hadv_tim ){
#ifndef LCD_USE_CUSTOM_CONFIG
	lcd_GPDMA_init();
#endif
	lcd_OSPI_init(hospi);
	lcd_TIM_init(hhalfline_tim, hdelay_tim, hadv_tim);
}
