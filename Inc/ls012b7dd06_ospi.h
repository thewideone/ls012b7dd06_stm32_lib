/*
 * ls012b7dd06_ospi.h
 *
 *  Created on: Feb 11, 2025
 *      Author: szyme
 */

#ifndef LS012B7DD06_STM32_INC_LS012B7DD06_OSPI_H_
#define LS012B7DD06_STM32_INC_LS012B7DD06_OSPI_H_

#include "stm32u5xx_hal.h"
#include "ls012b7dd06_config.h"

#ifdef LCD_USE_CUSTOM_CONFIG
void lcd_OSPI_init(OSPI_HandleTypeDef *hospi);
#else
void lcd_OSPI_init(void);
#endif

extern uint8_t lcd_tx_buf_1[OUT_DATA_BUF_SIZE];
extern HAL_StatusTypeDef lcd_OSPI_set_cmd_config(void);

//__weak void lcd_ospi_error_handler();

#endif /* LS012B7DD06_STM32_INC_LS012B7DD06_OSPI_H_ */
