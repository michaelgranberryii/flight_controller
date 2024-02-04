#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "sleep.h"
#include "axi_timer_ppm_receiver.h"
#include "axi_timer_pwm_motor.h"
#include "mpu_6050.h"
#include "bme_280.h"
#include "be_880.h"

int main()
{
    init_platform();

    // PPM
    tmr_ppm_init();

    // PWM
    trm_pwm_init_setup_all();

    // IMU
//    iic_imu_init();

    // BME
//    iic_bme_init();

    // GPS
    uart_be_init();

    // TODO Compass

    // GIC
    gicInit();

    xil_printf("Launching Flight Controller\n\r");
    while(1) {
//    	print_channel_time();
//    	update_pwm1(get_ch3());
//    	update_pwm2(get_ch3());
//    	update_pwm3(get_ch3());
//    	update_pwm4(get_ch3());
//    	MPU_Print_Results();
//    	read_bme();
//    	MPU_Calibration();
    	read_be_data();
    	usleep(1000*1000);
    }
    cleanup_platform();
    return 0;
}
