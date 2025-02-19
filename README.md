# LS012B7DD06 LCD driver library for STM32
An LS012B7DD06 driver for STM32 using OCTOSPI, timers, and GPIO. Still work in progress.

The code has been developed on the NUCLEO-U545RE-Q board with STM32U545 microcontroller.

Example usage at [this repo](https://github.com/thewideone/ls012b7dd06_stm32).

TODO:
- change lcd_colour_t to uint
- add draw line function
- correct LCD_setActive() and add LCD_setInactive()
- add LCD_deinit()
- get rid of debug printfs
- replace hardcoded registers and HAL macros
- add example hardware config (instead of configuring it in user program)
- add double buffering
- reconsider the busy flag in lcd_ctx_t
