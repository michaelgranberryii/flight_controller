#include <stdio.h>
#include "platform.h"
#include "xtmrctr.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xparameters.h"
#include "axi_timer_ppm_receiver.h"

XTmrCtr tmr_ppm;
XScuGic_Config *gic_config;
XScuGic gic;


u32 ch[NUM_OF_CH];
u32 pulse = 0;
float captured_count_value = 0;
float captured_time_Sec = 0;
float captured_time_mSec = 0;
u32 captured_time_uSec = 0;
u32 t1, t2, delta_t;
u32 too_long = 2100;

void tmr_ppm_init(){
	int status = 0;
	status = XTmrCtr_Initialize(&tmr_ppm, XPAR_AXI_TIMER_PPM_DEVICE_ID);
	if(status == XST_SUCCESS)
		xil_printf("TMR PPM Init Successful\n\r");
	else
		xil_printf("TMR PPM Init Failed\n\r");
	 status = XTmrCtr_SelfTest(&tmr_ppm, XTC_TIMER_0);
	if(status == XST_SUCCESS) {
		xil_printf("TMR PPM SELF TEST SUCCUSSFUL\n\r");
	} else {
		xil_printf("TMR PPM SELF TEST FAILED\n\r");
	}

}

void tmr_ppm_setup() {
	XTmrCtr_Stop(&tmr_ppm, XTC_TIMER_0);
	u32 Count = (100e6)*1;
	XTmrCtr_SetResetValue(&tmr_ppm, XTC_TIMER_0, Count);
	XTmrCtr_SetOptions(&tmr_ppm, XTC_TIMER_0, XTC_INT_MODE_OPTION | XTC_CAPTURE_MODE_OPTION);
	XTmrCtr_Start(&tmr_ppm, XTC_TIMER_0);
}

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

	// Connect and enable GIC
	XScuGic_Connect(&gic, XPAR_FABRIC_AXI_TIMER_PPM_INTERRUPT_INTR, (Xil_InterruptHandler)tmrHandler, &tmr_ppm);
	XScuGic_Enable(&gic, XPAR_FABRIC_AXI_TIMER_PPM_INTERRUPT_INTR);
}

void tmrHandler(){
	if (XTmrCtr_IsExpired(&tmr_ppm, XTC_TIMER_0)){
		captured_count_value = (float)XTmrCtr_GetCaptureValue(&tmr_ppm, XTC_TIMER_0);
		captured_time_Sec = (captured_count_value)*(1/100e6);
		captured_time_mSec = captured_time_Sec*1000;
		captured_time_uSec = (u32)(captured_time_mSec*1000);
		t1 = t2;
		t2 = captured_time_uSec;
		delta_t = (t2-t1);
//		xil_printf("delta_t: %d\n\r", delta_t);
		if (delta_t > too_long) {
			pulse = 0;
			ch[pulse] = delta_t;
			pulse = 1;
		} else {
			ch[pulse] = delta_t;
			pulse = (pulse + 1) % NUM_OF_CH;
		}
	}
	XTmrCtr_Reset(&tmr_ppm, XTC_TIMER_0);
}

u32 get_ch1() {
	return ch[1];
}

u32 get_ch2() {
	return ch[2];
}

u32 get_ch3() {
	return ch[3];
}

u32 get_ch4() {
	return ch[4];
}

void print_channel_time() {
	char channels[][20] = {"IDLE-HIGH", "ROLL", "PITCH", "THROTTLE", "YAW", "SWB", "VrB", "SWA", "VrA"};
	for (int i = 0; i < NUM_OF_CH; ++i) {
		(i != NUM_OF_CH-1) ? printf("%s[%d]: %lu\t", channels[i], i, ch[i]) : printf("%s[%d]: %lu\n\r", channels[i], i, ch[i]);
	}
}

