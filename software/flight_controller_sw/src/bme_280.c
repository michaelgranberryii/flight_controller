#include <stdio.h>
#include "bme_280.h"
#include "platform.h"
#include "xiic.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xparameters.h"
#include "sleep.h"
#include "iic.h"

IIC iic_bme;

void iic_bme_init() {
	iic_bme.chipAddr = BME_280_ADDRESS;
	iic_begin(&iic_bme, XPAR_AXI_IIC_0_DEVICE_ID, BME_280_ADDRESS);
	bme_280_setup();
}

void bme_280_setup() {
	bme_280_reset(0xB6);
	bme_280_read_id();
	bme_280_ctrl_hum();
	bme_280_ctrl_meas();
}

void bme_280_reset(u8 reset) {
	write_iic(&iic_bme, BME_280_RESET, &reset, 1);
}

void bme_280_read_id() {
	uint8_t bme_280_id = 0;
	read_iic(&iic_bme, BME_280_ID, &bme_280_id, 1);
	xil_printf("bme_280_id: %x\n\r", bme_280_id);
}

void bme_280_ctrl_hum() {
	u8 options = BME_280_OSRS_H1;
	write_iic(&iic_bme, BME_280_CTRL_HUM, &options, 1);
}

void bme_280_ctrl_meas() {
	u8 options = BME_280_OSRS_T1 | BME_280_OSRS_P1 | BME_280_NORMAL_MODE;
	write_iic(&iic_bme, BME_280_CTRL_MEAS, &options, 1);
}

void bme_280_read_hum() {
    uint8_t bme_280_hum_lsb = 0;
    uint8_t bme_280_hum_msb = 0;
    read_iic(&iic_bme, BME_280_HUM_LSB, &bme_280_hum_lsb, 1);
    read_iic(&iic_bme, BME_280_HUM_MSB, &bme_280_hum_msb, 1);
    xil_printf("bme_280_hum_lsb: %x\n\r", bme_280_hum_lsb);
    xil_printf("bme_280_hum_msb: %x\n\r", bme_280_hum_msb);
}

void bme_280_read_temp() {
    uint8_t bme_280_temp_xlsb = 0;
    uint8_t bme_280_temp_lsb = 0;
    uint8_t bme_280_temp_msb = 0;
    read_iic(&iic_bme, BME_280_TEMP_XLSB, &bme_280_temp_xlsb, 1);
    read_iic(&iic_bme, BME_280_TEMP_LSB, &bme_280_temp_lsb, 1);
    read_iic(&iic_bme, BME_280_TEMP_MSB, &bme_280_temp_msb, 1);
    xil_printf("bme_280_temp_xlsb: %x\n\r", bme_280_temp_xlsb);
    xil_printf("bme_280_temp_lsb: %x\n\r", bme_280_temp_lsb);
    xil_printf("bme_280_temp_msb: %x\n\r", bme_280_temp_msb);
}

void bme_280_read_press() {
    uint8_t bme_280_press_xlsb = 0;
    uint8_t bme_280_press_lsb = 0;
    uint8_t bme_280_press_msb = 0;
    read_iic(&iic_bme, BME_280_PRESS_XLSB, &bme_280_press_xlsb, 1);
    read_iic(&iic_bme, BME_280_PRESS_LSB, &bme_280_press_lsb, 1);
    read_iic(&iic_bme, BME_280_PRESS_MSB, &bme_280_press_msb, 1);
    xil_printf("bme_280_press_xlsb: %x\n\r", bme_280_press_xlsb);
    xil_printf("bme_280_press_lsb: %x\n\r", bme_280_press_lsb);
    xil_printf("bme_280_press_msb: %x\n\r", bme_280_press_msb);
}
