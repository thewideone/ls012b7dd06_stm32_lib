/*
 * ls012b7dd06_dma.c
 *
 *  Created on: Feb 11, 2025
 *      Author: szyme
 */

#include "ls012b7dd06_dma.h"

DMA_HandleTypeDef handle_GPDMA1_Channel12;

static void lcd_GPDMA_init(void){
	/* Peripheral clock enable */
	  __HAL_RCC_GPDMA1_CLK_ENABLE();

	  /* GPDMA1 interrupt Init */
	    HAL_NVIC_SetPriority(GPDMA1_Channel12_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(GPDMA1_Channel12_IRQn);
}
