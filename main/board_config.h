// board_config.h - movision (0223)
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
#include "driver/gpio.h"

// LCD Pin Definitions
#define PIN_NUM_LCD_CS    (GPIO_NUM_14)
#define PIN_NUM_LCD_PCLK  (GPIO_NUM_7)
#define PIN_NUM_LCD_DATA0 (GPIO_NUM_8)
#define PIN_NUM_LCD_DATA1 (GPIO_NUM_13)
#define PIN_NUM_LCD_DATA2 (GPIO_NUM_6)
#define PIN_NUM_LCD_DATA3 (GPIO_NUM_12)
#define PIN_NUM_LCD_RST   (GPIO_NUM_9)
#define PIN_NUM_LCD_VCI_EN (GPIO_NUM_18)

// Touch Pins
#define PIN_TOUCH_SDA GPIO_NUM_10
#define PIN_TOUCH_SCL GPIO_NUM_17
#define PIN_TOUCH_INT GPIO_NUM_11
#define PIN_TOUCH_RST GPIO_NUM_15
#endif
