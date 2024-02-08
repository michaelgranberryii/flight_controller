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

void gic_init(){
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
	be_880_interrupt_init(&gic);
}

int main()
{
    init_platform();

    // RGB GPIO
    rgb_led_init();

    // PPM
    ppm_init();

    // PWM
    pwm_init_all();

    // IMU
    mpu_6050_init();

    // BME
    bme_280_init();

    // GPS
    be_880_init();

    // TODO Compass

    // GIC
    gic_init();

    // Calibration
    rgb_led_set_output(RED_RGB_LED);
    xil_printf("Calibrating IMU...\n\r");
    mpu_calibration();
    xil_printf("Calibrating BME...\n\r");
    bme_280_alt_calibration();

    // Ready
    rgb_led_set_output(GREEN_RGB_LED);
    xil_printf("Launching Flight Controller\n\r");
    while(1) {
//    	print_channel_time();
//    	pwm_update_pwm1(get_ch1());
//    	pwm_update_pwm2(get_ch2());
//    	pwm_update_pwm3(get_ch3());
//    	pwm_update_pwm4(get_ch4());
//    	mpu_6050_print_results();
    	printf("Alt[cm]: %f, Press[hPa]: %f, Temp[C]: %0.2f, Hum[%%RH]: %0.3f\n\r",bme_280_get_altitude(), (((float)bme_280_get_corrected_press())/256)/100, ((float)bme_280_get_corrected_temp())/100, ((float)bme_280_get_corrected_hum())/1024);

//    	be_880_uart_loopback_test();
    	usleep(1000*50);
    }
    cleanup_platform();
    return 0;
}
