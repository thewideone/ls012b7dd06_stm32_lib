/*
 * ls012b7dd06_stm32.h
 *
 *  Created on: Feb 6, 2025
 *      Author: szyme
 */

#ifndef LS012B7DD06_STM32_INC_LS012B7DD06_H_
#define LS012B7DD06_STM32_INC_LS012B7DD06_H_

#include "stm32u5xx_hal.h"
#include "ls012b7dd06_config.h"
#include "ls012b7dd06_dma.h"
#include "ls012b7dd06_ospi.h"
#include "ls012b7dd06_tim.h"

void lcd_init(OSPI_HandleTypeDef *hospi);

#endif /* LS012B7DD06_STM32_INC_LS012B7DD06_H_ */
