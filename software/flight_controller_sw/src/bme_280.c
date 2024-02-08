#include "bme_280.h"

IIC bme;

// Define the data types for BME280
typedef int32_t BME280_S32_t;
typedef uint32_t BME280_U32_t;
typedef int64_t BME280_S64_t;

// Global variable for fine temperature
BME280_S32_t t_fine;
u8 is_cal = 0;
u16 dig_T1, dig_P1;
s16 dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
s16 dig_P6, dig_P7, dig_P8, dig_P9;

unsigned char dig_H1, dig_H3, dig_H6;
s16 dig_H2, dig_H4, dig_H5;

float altitude_barometer, altitude_barometer_start_up;
int rate_calibration__number;

void bme_280_init() {
	bme.chipAddr = BME_280_ADDRESS;
	iic_begin(&bme, XPAR_AXI_IIC_0_DEVICE_ID, BME_280_ADDRESS);
	bme_280_setup();
}

void bme_280_setup() {
	bme_280_trim_param();
	bme_280_reset(0xB6);
	bme_280_read_id();
	bme_280_ctrl_hum();
	bme_280_ctrl_meas();

}

void bme_280_reset(u8 reset) {
	iic_write(&bme, BME_280_RESET, &reset, 1);
}

void bme_280_read_id() {
	uint8_t bme_280_id = 0;
	iic_read(&bme, BME_280_ID, &bme_280_id, 1);
	xil_printf("BME280 ID: %x\n\r", bme_280_id);
}

void bme_280_ctrl_hum() {
	u8 options = BME_280_OSRS_H1;
	iic_write(&bme, BME_280_CTRL_HUM, &options, 1);
}

void bme_280_ctrl_meas() {
	u8 options = BME_280_OSRS_T2 | BME_280_OSRS_P16 | BME_280_NORMAL_MODE;
	iic_write(&bme, BME_280_CTRL_MEAS, &options, 1);
}

void bme_280_config() {
	u8 options = 0x14;
	iic_write(&bme, BME_280_CONFIG, &options, 1);

}


void bme_280_trim_param() {
	u8 data[33], i=0;
	int addr;
	for(addr = 0x88; addr <= 0x9F; addr++) {
		iic_read(&bme, addr, &data[i], 1);
		i++;
	}
	addr = 0xA1;
	iic_read(&bme, addr, &data[i], 1);
	i++;
	for(addr = 0xE1; addr <= 0xE6; addr++) {
		iic_read(&bme, addr, &data[i], 1);
		i++;
	}

	  dig_T1 = (data[1] << 8) | data[0];
	  dig_T2 = (data[3] << 8) | data[2];
	  dig_T3 = (data[5] << 8) | data[4];
	  dig_P1 = (data[7] << 8) | data[6];
	  dig_P2 = (data[9] << 8) | data[8];
	  dig_P3 = (data[11]<< 8) | data[10];
	  dig_P4 = (data[13]<< 8) | data[12];
	  dig_P5 = (data[15]<< 8) | data[14];
	  dig_P6 = (data[17]<< 8) | data[16];
	  dig_P7 = (data[19]<< 8) | data[18];
	  dig_P8 = (data[21]<< 8) | data[20];
	  dig_P9 = (data[23]<< 8) | data[22];
	  dig_H1 = (data[25]<< 8) | data[24];
	  dig_H2 = (data[27]<< 8) | data[26];
	  dig_H3 = (data[29]<< 8) | data[28];
	  dig_H4 = (data[31]<< 8) | data[30];
	  dig_H5 = (data[33]<< 8) | data[32];
	  usleep(1000*250);
}

BME280_U32_t bme_280_read_raw_hum() {
    u8 hum_lsb;
    u8 hum_msb;

    iic_read(&bme, BME_280_HUM_LSB, &hum_lsb, 1);
    iic_read(&bme, BME_280_HUM_MSB, &hum_msb, 1);

    u32 hum_lsb_u32 = hum_lsb;
    u32 hum_msb_u32 = hum_msb;

    BME280_U32_t adc_H = (hum_lsb_u32 << 8) | hum_msb_u32;
    return adc_H;

}

uint32_t bme_280_get_corrected_hum() {
	BME280_S32_t adc_H = bme_280_read_raw_hum();
    BME280_U32_t hum = bme_280_compensate_H_int32(adc_H);
    return hum;
}

BME280_U32_t bme_280_read_raw_temp() {
    u8 temp_xlsb;
    u8 temp_lsb;
    u8 temp_msb;

    iic_read(&bme, BME_280_TEMP_XLSB, &temp_xlsb, 1);
    iic_read(&bme, BME_280_TEMP_LSB, &temp_lsb, 1);
    iic_read(&bme, BME_280_TEMP_MSB, &temp_msb, 1);

    u32 temp_xlsb_u32 = temp_xlsb;
    u32 temp_lsb_u32 = temp_lsb;
    u32 temp_msb_u32 = temp_msb;

    BME280_U32_t adc_T = (temp_msb_u32 << 12) | (temp_lsb_u32 << 4) | (temp_xlsb_u32 >> 4);
    return adc_T;
}

uint32_t bme_280_get_corrected_temp() {
	BME280_S32_t adc_T = bme_280_read_raw_temp();
    BME280_U32_t temp = bme_280_compensate_T_int32(adc_T);
    return temp;
}

BME280_U32_t bme_280_read_raw_press() {
    u8 press_xlsb;
    u8 press_lsb;
    u8 press_msb;

    iic_read(&bme, BME_280_PRESS_XLSB, &press_xlsb, 1);
    iic_read(&bme, BME_280_PRESS_LSB, &press_lsb, 1);
    iic_read(&bme, BME_280_PRESS_MSB, &press_msb, 1);

    u32 press_xlsb_u32 = press_xlsb;
    u32 press_lsb_u32 = press_lsb;
    u32 press_msb_u32 = press_msb;

    BME280_U32_t adc_P = (press_msb_u32 << 12) | (press_lsb_u32 << 4) | (press_xlsb_u32 >> 4);
    return adc_P;
}

uint32_t bme_280_get_corrected_press() {
	BME280_U32_t adc_P = bme_280_read_raw_press();
    BME280_U32_t press = bme_280_compensate_p_int64(adc_P);
    return press;
}

float bme_280_get_altitude() {
	double pressure = (double)bme_280_get_corrected_press()/100;
	altitude_barometer = 44330*(1-pow(pressure/1013.25, 1/5.255))*100;
	if(is_cal) {
		altitude_barometer-= altitude_barometer_start_up;
		return altitude_barometer;
	} else {
		return altitude_barometer;
	}

}

void bme_280_alt_calibration() {
	for(rate_calibration__number = 0; rate_calibration__number < 2000; rate_calibration__number++) {
		bme_280_get_altitude();
		altitude_barometer_start_up+=altitude_barometer;
		usleep(1000*1);
	}
	altitude_barometer_start_up/=2000;
	is_cal = 1;
}


// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BME280_U32_t bme_280_compensate_p_int64(BME280_S32_t adc_P)
{
    BME280_S64_t var1, var2, p;

    // Calculate compensation according to the provided formula
    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)dig_P6;
    var2 = var2 + ((var1 * (BME280_S64_t)dig_P5) << 17);
    var2 = var2 + (((BME280_S64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (BME280_S64_t)dig_P3) >> 8) + ((var1 * (BME280_S64_t)dig_P2) << 12);
    var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((BME280_S64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((BME280_S64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)dig_P7) << 4);

    return (BME280_U32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BME280_U32_t bme_280_compensate_H_int32(BME280_S32_t adc_H)
{
    BME280_S32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) * v_x1_u32r)) +
                   ((BME280_S32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)dig_H3)) >> 11) +
                                                                      ((BME280_S32_t)32768))) >>
                     10) +
                    ((BME280_S32_t)2097152)) *
                   ((BME280_S32_t)dig_H2) +
                   8192) >>
                 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (BME280_U32_t)(v_x1_u32r >> 12);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
BME280_S32_t bme_280_compensate_T_int32(BME280_S32_t adc_T)
{
    BME280_S32_t var1, var2, T;

    // Calculation according to the compensation formula
    var1 = ((((adc_T>>3) - ((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) * ((BME280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;

    // Temperature calculation with resolution 0.01 DegC
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

