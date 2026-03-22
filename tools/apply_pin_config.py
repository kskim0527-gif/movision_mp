import sys
import os

BOARD_CONFIG_FILE = "main/board_config.h"

WS_PINS = """// board_config.h - movision_ws
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
"""

MOVISION_PINS = """// board_config.h - movision
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
"""

def apply_config(board_type):
    if board_type == "ws":
        content = WS_PINS
    elif board_type == "movision":
        content = MOVISION_PINS
    else:
        print(f"Unknown board type: {board_type}")
        return False
    
    with open(BOARD_CONFIG_FILE, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"Applied {board_type} pin configuration to {BOARD_CONFIG_FILE}")
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python apply_pin_config.py [ws|movision]")
    else:
        apply_config(sys.argv[1])
