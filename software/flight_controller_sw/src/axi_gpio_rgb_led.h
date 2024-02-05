#ifndef AXI_GPIO_RGB_LED   /* prevent circular inclusions */
#define AXI_GPIO_RGB_LED

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xgpio.h"
#include "xparameters.h"
#include "sleep.h"

#define GPIO_CHANNEL 1
#define GPIO_PIN_DIR_OUT (0x7 & 0x0)
#define LED_SLEEP_US 300000

#define RED_RGB_LED 0x4
#define GREEN_RGB_LED 0x2
#define BLUE_RGB_LED 0x1
#define WHITE_RGB_LED 0x7
#define OFF_RGB_LED 0x4

void gpio_rgb_init(void);
void blink_led(u32 rgb_color);

#endif  /* end of protection macro */
