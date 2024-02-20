#ifndef IST8310   /* prevent circular inclusions */
#define IST8310

#include <stdio.h>
#include "platform.h"
#include "xiic.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xparameters.h"
#include "iic.h"
#include "sleep.h"

#define IST8310_ADDRESS                        	0x0D

#define IST8310_WHO_AM_I              				0x0

#define IST8310_STATUS_1                  			0x02
#define IST8310_STATUS_2                  			0x09

#define IST8310_OUTPUT_VALUE_X_L                   	0x03
#define IST8310_OUTPUT_VALUE_X_H                   	0x04

#define IST8310_OUTPUT_VALUE_Y_L                   	0x05
#define IST8310_OUTPUT_VALUE_Y_H                   	0x06

#define IST8310_OUTPUT_VALUE_Z_L                   	0x07
#define IST8310_OUTPUT_VALUE_Z_H                   	0x08

#define IST8310_CONTROL_1                  			0x0A
#define IST8310_CONTROL_1_SINGLE_MEAS_MODE 			0x1

#define IST8310_CONTROL_2                  			0x0B
#define IST8310_CONTROL_2_DREM_EN          			0x8
#define IST8310_CONTROL_2_DRP_ACTIVE_HIGH 			0x4
#define IST8310_CONTROL_2_SRST			 			0x1

#define IST8310_OUTPUT_VALUE_T_L                   	0x0C
#define IST8310_OUTPUT_VALUE_T_H                   	0x0D

void ist8310_init();
void ist8310_setup();
void ist8310_standy_by();
void ist8310_reset();
void ist8310_device_id();
void ist8310_cnrl_1();
void ist8310_cnrl_2();
void ist8310_status_1();
u8 ist8310_status_2();
void ist8310_self_test();

void ist8310_print_x();
void ist8310_print_y();
void ist8310_print_z();
void ist8310_print_t();

uint8_t ist8310_x_high_byte();
uint8_t ist8310_x_low_byte();
uint8_t ist8310_y_high_byte();
uint8_t ist8310_y_low_byte();
uint8_t ist8310_z_high_byte();
uint8_t ist8310_z_low_byte();
uint8_t ist8310_t_high_byte();
uint8_t ist8310_t_low_byte();

#endif  /* end of protection macro */
