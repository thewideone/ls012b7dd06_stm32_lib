/*
 * ls012b7dd06.h
 *
 *  Created on: Feb 6, 2025
 *      Author: thewideone
 */

#ifndef _LS012B7DD06_H_
#define _LS012B7DD06_H_

#include "stm32u5xx_hal.h"
#include "ls012b7dd06_config.h"
#include <stdbool.h>

typedef struct {
	OSPI_HandleTypeDef *hospi;
	OSPI_RegularCmdTypeDef ospi_cmd;
	TIM_HandleTypeDef *hhalfline_tim;
	TIM_HandleTypeDef *hdelay_tim;
	TIM_HandleTypeDef *hadv_tim;
	TIM_HandleTypeDef *hpwr_tim;
	GPIO_TypeDef *hintb_port;
	uint16_t intb_pin;
//	bool active;
	uint8_t *buf1;
} lcd_init_t;

typedef struct {
	OSPI_HandleTypeDef *hospi;
	OSPI_RegularCmdTypeDef ospi_cmd;
	TIM_HandleTypeDef *hhalfline_tim;
	TIM_HandleTypeDef *hdelay_tim;
	TIM_HandleTypeDef *hadv_tim;
	TIM_HandleTypeDef *hpwr_tim;
	GPIO_TypeDef *hintb_port;
	uint16_t intb_pin;
	bool active;
	bool busy;
	uint8_t *buf1;
} lcd_ctx_t;

void LCD_init(uint8_t instance_no, lcd_init_t *init);
HAL_StatusTypeDef LCD_PWM_power_enable(void);
HAL_StatusTypeDef LCD_PWM_power_disable(void);
void LCD_setActive(uint8_t instance_no);

HAL_StatusTypeDef LCD_displayFrame(void);
void LCD_cls(void);
void LCD_setPixel(int16_t x, int16_t y, lcd_colour_t colour);
void LCD_setColour(lcd_colour_t colour);
void LCD_drawSquare(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, lcd_colour_t colour);
void LCD_drawHLine(uint16_t x0, uint16_t x1, uint16_t y, lcd_colour_t colour);
void LCD_drawLine( int16_t x0, int16_t y0, int16_t x1, int16_t y1, lcd_colour_t color );
void LCD_drawTestFigure(void);

#endif /* _LS012B7DD06_H_ */
