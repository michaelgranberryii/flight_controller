#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "sleep.h"
#include "xil_exception.h"
#include "xscugic.h"

#include "axi_gpio_rgb_led.h"
#include "axi_timer_ppm_receiver.h"
#include "axi_timer_pwm_motor.h"
#include "mpu_6050.h"
#include "bme_280.h"
#include "be_880.h"

XScuGic_Config *gic_config;
XScuGic gic;

void gicInit(){
	int status = 0;
	gic_config = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);

	status = XScuGic_CfgInitialize(&gic, gic_config, gic_config->CpuBaseAddress);
	if(status == XST_SUCCESS)
		xil_printf("GIC Init Successful\n\r");
	else
		xil_printf("GIC Init Failed\n\r");

	// Init and enable exception
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT, (Xil_ExceptionHandler) XScuGic_InterruptHandler,	&gic);
	Xil_ExceptionEnable();

	trm_ppm_intr_init(&gic);
	uart_be_intr_init(&gic);
}

int main()
{
    init_platform();

    // RGB GPIO
    gpio_rgb_init();

    // PPM
    tmr_ppm_init();

    // PWM
    trm_pwm_init_setup_all();

    // IMU
    iic_imu_init();

    // BME
    iic_bme_init();

    // GPS
    uart_be_init();

    // TODO Compass

    // GIC
    gicInit();

    // Calibration
    xil_printf("Calibrating IMU...\n\r");
    blink_led(RED_RGB_LED);
    MPU_Calibration();

    // Ready
    blink_led(GREEN_RGB_LED);
    xil_printf("Launching Flight Controller\n\r");
    while(1) {
    	print_channel_time();
    	update_pwm1(get_ch1());
    	update_pwm2(get_ch2());
    	update_pwm3(get_ch3());
    	update_pwm4(get_ch4());
    	MPU_Print_Results();
    	bme_280_read_hum();
    	bme_280_read_temp();
    	bme_280_read_press();
    	uart_loopback_test();
    	usleep(1000*20);
    }
    cleanup_platform();
    return 0;
}
