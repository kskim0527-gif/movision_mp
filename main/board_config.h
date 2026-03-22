// board_config.h - movision_ws
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
#include "driver/gpio.h"

// LCD Pin Definitions
#define PIN_NUM_LCD_CS    (GPIO_NUM_12)
#define PIN_NUM_LCD_PCLK  (GPIO_NUM_38)
#define PIN_NUM_LCD_DATA0 (GPIO_NUM_4)
#define PIN_NUM_LCD_DATA1 (GPIO_NUM_5)
#define PIN_NUM_LCD_DATA2 (GPIO_NUM_6)
#define PIN_NUM_LCD_DATA3 (GPIO_NUM_7)
#define PIN_NUM_LCD_RST   (GPIO_NUM_39)
#define PIN_NUM_LCD_VCI_EN (GPIO_NUM_18)

// Touch Pins
#define PIN_TOUCH_SDA GPIO_NUM_15
#define PIN_TOUCH_SCL GPIO_NUM_14
#define PIN_TOUCH_INT GPIO_NUM_11
#define PIN_TOUCH_RST GPIO_NUM_40
#endif
