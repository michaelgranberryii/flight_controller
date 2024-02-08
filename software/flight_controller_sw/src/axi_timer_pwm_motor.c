#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xtmrctr.h"
#include "xparameters.h"
#include "axi_timer_pwm_motor.h"

int current_pulse_width_nSec = 0;
int previous_pulse_width_nSec = 0;

XTmrCtr pwm1, pwm2, pwm3, pwm4;

void pwm_init_all() {
	pwm_init(&pwm1, XPAR_AXI_TIMER_PWM_1_DEVICE_ID);
	pwm_init(&pwm2, XPAR_AXI_TIMER_PWM_2_DEVICE_ID);
	pwm_init(&pwm3, XPAR_AXI_TIMER_PWM_3_DEVICE_ID);
	pwm_init(&pwm4, XPAR_AXI_TIMER_PWM_4_DEVICE_ID);
}

void pwm_init(XTmrCtr *tmr_pwm_ptr, u16 DeviceID) {
	int status = XTmrCtr_Initialize(tmr_pwm_ptr, DeviceID);
	if(status == XST_SUCCESS) {
		xil_printf("TMR PWM%u INIT SUCCUSSFUL\n\r", DeviceID);
	} else {
		xil_printf("TMR PWM%u INIT FAILED\n\r", DeviceID);
	}
	status = XTmrCtr_SelfTest(tmr_pwm_ptr, XTC_TIMER_0);
	if(status == XST_SUCCESS) {
		xil_printf("TMR PWM%u  SELF TEST SUCCUSSFUL\n\r", DeviceID);
	} else {
		xil_printf("TMR PWM%u  SELF TEST FAILED\n\r", DeviceID);
	}

	pwm_setup(tmr_pwm_ptr);
}

void pwm_setup(XTmrCtr *tmr_pwm_ptr) {
	XTmrCtr_Stop(tmr_pwm_ptr, XTC_TIMER_0);
	u32 tmrctr0_option = XTmrCtr_GetOptions(tmr_pwm_ptr, XTC_TIMER_0);
	tmrctr0_option |= XTC_DOWN_COUNT_OPTION;
	XTmrCtr_SetOptions(tmr_pwm_ptr, XTC_TIMER_0, tmrctr0_option);
	XTmrCtr_PwmConfigure(tmr_pwm_ptr, PWM_PERIOD_nSEC, ONE_mSec);
	XTmrCtr_PwmEnable(tmr_pwm_ptr);
	XTmrCtr_Start(tmr_pwm_ptr, XTC_TIMER_0);
}

// Function to map a value from one range to another
int pwm_map_range(float value, float in_min, float in_max, float out_min, float out_max) {
	float result = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return (int)result;
}

void pwm_update_pwm1(int input_channel_value) {
//	xil_printf("input_channel1_value: %d\n\r", input_channel_value);
	pwm_update(&pwm1, input_channel_value);
}

void pwm_update_pwm2(int input_channel_value) {
//	xil_printf("input_channel2_value: %d\n\r", input_channel_value);
	pwm_update(&pwm2, input_channel_value);
}

void pwm_update_pwm3(int input_channel_value) {
//	xil_printf("input_channel3_value: %d\n\r", input_channel_value);
	pwm_update(&pwm3, input_channel_value);
}

void pwm_update_pwm4(int input_channel_value) {
//	xil_printf("input_channel4_value: %d\n\r", input_channel_value);
	pwm_update(&pwm4, input_channel_value);
}

void pwm_update(XTmrCtr *tmr_pwm_ptr, int input_channel_value) {
//	xil_printf("Switch value : %d\n\r", input_channel_value);
	current_pulse_width_nSec = pwm_map_range((float)input_channel_value, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
//	xil_printf("Current duty cycle value : %d nSec\n\r", (current_pulse_width_nSec));
//	if(previous_pulse_width_nSec != current_pulse_width_nSec){
		pwm_set_new_duty_cycle(tmr_pwm_ptr, current_pulse_width_nSec);
//	}
	previous_pulse_width_nSec = current_pulse_width_nSec;
}

void pwm_set_new_duty_cycle(XTmrCtr *tmr_pwm_ptr, int pulse_width_nSec) {
	XTmrCtr_PwmDisable(tmr_pwm_ptr);
	XTmrCtr_PwmConfigure(tmr_pwm_ptr, PWM_PERIOD_nSEC, pulse_width_nSec);
	XTmrCtr_PwmEnable(tmr_pwm_ptr);
}
