#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "axi_timer_ppm_receiver.h"
#include "axi_timer_pwm_motor.h"
#include "axi_iic_imu.h"
#include "sleep.h"


int main()
{
    init_platform();

    // PPM
    tmr_ppm_init();
    tmr_ppm_setup();

    // PWM
    trm_pwm_init_setup_all();

    // IMU
    iic_imu_init();
    iic_imu_setup();

    // GIC
    gicInit();

    xil_printf("Launching Flight Controller\n\r");
    while(1) {
//    	print_channel_time();
//    	update_pwm1(get_ch3());
//    	update_pwm2(get_ch3());
//    	update_pwm3(get_ch3());
//    	update_pwm4(get_ch3());
    	MPU_Print_Results();
//    	MPU_Calibration();
    	usleep(1000*20);
    }
    cleanup_platform();
    return 0;
}
