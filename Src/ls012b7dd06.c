/*
 * ls012b7dd06_stm32.c
 *
 *  Created on: Feb 6, 2025
 *      Author: szyme
 */

#include "ls012b7dd06.h"

void lcd_init(OSPI_HandleTypeDef *hospi){
#ifndef LCD_USE_CUSTOM_CONFIG
	lcd_GPDMA_init();
#endif
	lcd_OSPI_init(hospi);
}
