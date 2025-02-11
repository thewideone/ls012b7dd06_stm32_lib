/*
 * ls012b7dd06_stm32.c
 *
 *  Created on: Feb 6, 2025
 *      Author: szyme
 */

#include "ls012b7dd06.h"

void lcd_init(void){
	lcd_GPDMA_init();
	lcd_OSPI_init();
}
