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
	iic_begin(&iic_bme, XPAR_AXI_IIC_0_DEVICE_ID, BME_280_ADDRESS);
	iic_bme_setup();
}

void iic_bme_setup() {

}

void read_bme() {
    uint8_t bme_280_hum_lsb;
    read_iic(&iic_bme, BME_280_HUM_LSB, &bme_280_hum_lsb, 1);
}
