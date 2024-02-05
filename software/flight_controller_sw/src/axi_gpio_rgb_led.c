/**
 * @author Michael Granberry
 * @brief Simple AXI GPIO example. LED4 is controlled my BTN4.
 * @file main.c
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include "axi_gpio_rgb_led.h"



XGpio_Config *gpio_config;
XGpio gpio;

void gpio_rgb_init(void) {
	int status = XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_DEVICE_ID);
	if(status == XST_SUCCESS) {
		xil_printf("AXI GPIO INIT SUCCESSFUL\n\r");
	} else {
		xil_printf("AXI GPIO INIT FAILED\n\r");
	}

	XGpio_SetDataDirection(&gpio, GPIO_CHANNEL, GPIO_PIN_DIR_OUT);
}

void blink_led(u32 rgb_color) {
	rgb_color = (0x7 & rgb_color);
	XGpio_DiscreteWrite(&gpio, GPIO_CHANNEL, rgb_color);
}

