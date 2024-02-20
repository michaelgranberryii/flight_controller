#include "ist8310.h"

IIC ist8310;

void ist8310_init() {
	iic_begin(&ist8310, XPAR_AXI_IIC_0_DEVICE_ID, IST8310_ADDRESS);
	ist8310_setup();
}

void ist8310_setup() {
	ist8310_reset();
//	ist8310_self_test();
	ist8310_standy_by();
	ist8310_device_id();
	ist8310_cnrl_2();
}

void ist8310_reset() {
    u8 option = IST8310_CONTROL_2_SRST;
	iic_write(&ist8310, IST8310_CONTROL_2, &option, 1);
}

void ist8310_standy_by() {
    u8 option = 0x0;
	iic_write(&ist8310, IST8310_CONTROL_1, &option, 1);
	option = 0xC0;
	iic_write(&ist8310, 0x42, &option, 1);
}

void ist8310_device_id() {
	u8 who_am_i = 10;
	iic_read(&ist8310, IST8310_WHO_AM_I, &who_am_i, 1);
	xil_printf("who_am_i: %x\n\r", who_am_i);
}

void ist8310_cnrl_1() {
    u8 option = IST8310_CONTROL_1_SINGLE_MEAS_MODE;
	iic_write(&ist8310, IST8310_CONTROL_1, &option, 1);
	iic_read(&ist8310, IST8310_CONTROL_1, &option, 1);
}

void ist8310_cnrl_2() {
    u8 option = IST8310_CONTROL_2_DREM_EN | IST8310_CONTROL_2_DRP_ACTIVE_HIGH;
	iic_write(&ist8310, IST8310_CONTROL_2, &option, 1);
	iic_read(&ist8310, IST8310_CONTROL_2, &option, 1);
}

void ist8310_status_1() {
	u8 is_data_ready = 0;
	while(!(0xE & is_data_ready)) {
		iic_read(&ist8310, IST8310_STATUS_1, &is_data_ready, 1);
	}
}

u8 ist8310_status_2() {
	u8 intr = 0;
	iic_read(&ist8310, IST8310_STATUS_2, &intr, 1);
	return intr;
}

void ist8310_self_test() {
	u8 data = 0x40;
	iic_write(&ist8310, 0x0C, &data, 1);
	ist8310_print_x();
	data = 0x00;
	iic_write(&ist8310, 0x0C, &data, 1);
	ist8310_print_x();
}

void ist8310_print_x() {
	u8 h = ist8310_x_high_byte();
	u8 l = ist8310_x_low_byte();

	u16 h_16 = h;
	u16 l_16 = l;

	s16 x = ((0xFF & h_16) << 8) | (0XFF & l_16);
	xil_printf("h: %u, l: %u, x: %d\n\r", h, l, x);
}

void ist8310_print_y() {
	u8 h = ist8310_y_high_byte();
	u8 l = ist8310_y_low_byte();
	u16 h_16 = h;
	u16 l_16 = l;

	s16 y = ((0xFF & h_16) << 8) | (0XFF & l_16);
	xil_printf("h: %u, l: %u, y: %d\n\r", h, l, y);
}

void ist8310_print_z() {
	u8 h = ist8310_z_high_byte();
	u8 l = ist8310_z_low_byte();
	u16 h_16 = h;
	u16 l_16 = l;

	s16 z = ((0xFF & h_16) << 8) | (0XFF & l_16);
	xil_printf("h: %u, l: %u, z: %d\n\r", h, l, z);
}

void ist8310_print_t() {
	u8 h = ist8310_t_high_byte();
	u8 l = ist8310_t_low_byte();
	u16 h_16 = h;
	u16 l_16 = l;

	s16 t = ((0xFF & h_16) << 8) | (0XFF & l_16);
	xil_printf("h: %u, l: %u, t: %d\n\r", h, l, t);
}

uint8_t ist8310_x_high_byte()
{
    uint8_t xout_l;
    iic_read(&ist8310, IST8310_OUTPUT_VALUE_X_H, &xout_l, 1);
    return xout_l;
}

uint8_t ist8310_x_low_byte()
{
    uint8_t xout_l;
    iic_read(&ist8310, IST8310_OUTPUT_VALUE_X_L, &xout_l, 1);
    return xout_l;
}

uint8_t ist8310_y_high_byte()
{
    uint8_t yout_h;
    iic_read(&ist8310, IST8310_OUTPUT_VALUE_Y_H, &yout_h, 1);
    return yout_h;
}

uint8_t ist8310_y_low_byte()
{
    uint8_t yout_l;
    iic_read(&ist8310, IST8310_OUTPUT_VALUE_Y_L, &yout_l, 1);
    return yout_l;
}

uint8_t ist8310_z_high_byte()
{
    uint8_t zout_h;
    iic_read(&ist8310, IST8310_OUTPUT_VALUE_Z_H, &zout_h, 1);
    return zout_h;
}

uint8_t ist8310_z_low_byte()
{
    uint8_t zout_l;
    iic_read(&ist8310, IST8310_OUTPUT_VALUE_Z_L, &zout_l, 1);
    return zout_l;
}

uint8_t ist8310_t_high_byte() {
    uint8_t tout_h;
    iic_read(&ist8310, IST8310_OUTPUT_VALUE_T_H, &tout_h, 1);
    return tout_h;
}

uint8_t ist8310_t_low_byte() {
    uint8_t tout_l;
    iic_read(&ist8310, IST8310_OUTPUT_VALUE_T_L, &tout_l, 1);
    return tout_l;
}

