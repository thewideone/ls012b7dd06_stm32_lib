/*
 * lcd_config.h
 *
 *  Created on: Jul 24, 2024
 *      Author: szyme
 */

#ifndef LS012B7DD06_STM32_INC_LS012B7DD06_CONFIG_H_
#define LS012B7DD06_STM32_INC_LS012B7DD06_CONFIG_H_

//
// This file contains only definitions and structures
// to avoid circular dependencies.
//

#define LCD_USE_CUSTOM_CONFIG

//
// Colours
//

// Display colour depth (in bits per channel).
// Add a possibility to make it 1 or monochrome?
#define RLCD_COLOUR_DEPTH 2

//
// Colour format:
//  7 6 5 4 3 2 1 0 <- bit no.
//  _______________
// |-|-|B|B|G|G|R|R|
//
typedef union {
    uint8_t val;
    struct {
        uint8_t r_msb       : 1;
        uint8_t r_lsb       : 1;
        uint8_t g_msb       : 1;
        uint8_t g_lsb       : 1;
        uint8_t b_msb       : 1;
        uint8_t b_lsb       : 1;
        uint8_t reserved    : 2;
    } bits;
} lcd_colour_t;

// Colour bit positions from the above struct
#define COLOUR_R_LSB_BIT 0
#define COLOUR_R_MSB_BIT 1
#define COLOUR_G_LSB_BIT 2
#define COLOUR_G_MSB_BIT 3
#define COLOUR_B_LSB_BIT 4
#define COLOUR_B_MSB_BIT 5

// Colour bit masks
#define COLOUR_R_MSB_MASK ( 1 << COLOUR_R_MSB_BIT )
#define COLOUR_R_LSB_MASK ( 1 << COLOUR_R_LSB_BIT )
#define COLOUR_G_MSB_MASK ( 1 << COLOUR_G_MSB_BIT )
#define COLOUR_G_LSB_MASK ( 1 << COLOUR_G_LSB_BIT )
#define COLOUR_B_MSB_MASK ( 1 << COLOUR_B_MSB_BIT )
#define COLOUR_B_LSB_MASK ( 1 << COLOUR_B_LSB_BIT )

// Basic colours
#define COLOUR_WHITE   0x3F	// all 1's except the reserved bits
#define COLOUR_BLACK   0x00
#define COLOUR_RED     ( COLOUR_R_MSB_MASK | COLOUR_R_LSB_MASK )
#define COLOUR_GREEN   ( COLOUR_G_MSB_MASK | COLOUR_G_LSB_MASK )
#define COLOUR_BLUE    ( COLOUR_B_MSB_MASK | COLOUR_B_LSB_MASK )

#define COLOUR_CYAN    (COLOUR_GREEN | COLOUR_BLUE)
#define COLOUR_MAGENTA (COLOUR_RED   | COLOUR_BLUE)
#define COLOUR_YELLOW  (COLOUR_RED   | COLOUR_GREEN)

//
// Hardware, display dimensions and buffer
//

// Display width
#define RLCD_DISP_W 240
// Display height
#define RLCD_DISP_H 240

// Display frame buffer size (in bytes).
// Add correction for a round display or leave dummy bits as they are.
#define RLCD_BUF_SIZE (RLCD_DISP_W * RLCD_DISP_H) * sizeof(lcd_colour_t) //* RLCD_COLOUR_DEPTH / 8
#define BCK_TRAILING_DUMMY_EDGE_CNT 2
#define OUT_DATA_BUF_LINE_W ( RLCD_DISP_W/2 + BCK_TRAILING_DUMMY_EDGE_CNT )
#define OUT_DATA_BUF_SIZE ( RLCD_DISP_H*2 * OUT_DATA_BUF_LINE_W )

#define GPIO_BUS_R0_BIT 0
#define GPIO_BUS_R1_BIT 1
#define GPIO_BUS_G0_BIT 2
#define GPIO_BUS_G1_BIT 3
#define GPIO_BUS_B0_BIT 4
#define GPIO_BUS_B1_BIT 5
#define GPIO_BUS_BSP_BIT 6
// the last bit is not used `\(c:)/`

//
// Graphics
//

// Maximum number of characters in a single string
#define RLCD_STR_MAX_CHAR_CNT ( RLCD_DISP_W / 5 )

#endif /* LS012B7DD06_STM32_INC_LS012B7DD06_CONFIG_H_ */
