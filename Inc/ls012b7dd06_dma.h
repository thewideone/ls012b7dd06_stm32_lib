/*
 * ls012b7dd06_dma.h
 *
 *  Created on: Feb 11, 2025
 *      Author: szyme
 */

#ifndef LS012B7DD06_STM32_INC_LS012B7DD06_DMA_H_
#define LS012B7DD06_STM32_INC_LS012B7DD06_DMA_H_

#include "stm32u5xx_hal.h"
#include "ls012b7dd06_config.h"

#ifndef LCD_USE_CUSTOM_CONFIG
extern DMA_HandleTypeDef handle_GPDMA1_Channel12;


//#ifndef LCD_USE_CUSTOM_CONFIG
static void lcd_GPDMA_init(DMA_HandleTypeDef *hgpdma);
//#endif
#endif

#endif /* LS012B7DD06_STM32_INC_LS012B7DD06_DMA_H_ */
