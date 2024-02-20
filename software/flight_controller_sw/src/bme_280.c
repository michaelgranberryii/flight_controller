#include "bme_280.h"

IIC bme;
BME280_S32_t t_fine;
const float sea_level_press_hPa = 1017;
unsigned short dig_T1, dig_P1;
signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
signed short dig_P6, dig_P7, dig_P8, dig_P9;
unsigned char dig_H1, dig_H3, dig_H6;
signed short dig_H2, dig_H4, dig_H5;
float altitude_barometer_start_up = 0;

void bme_280_init() {
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
}

void bme_280_ctrl_hum() {
	u8 options = BME_280_OSRS_H2;
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
	bme_280_trim_param_temp();
	usleep(1000*250);
	bme_280_trim_param_press();
	usleep(1000*250);
	bme_280_trim_param_hum();
	usleep(1000*250);
}

void bme_280_trim_param_temp() {
	u8 data_temp[6] = {0};
	u8 i = 0;
	for(int addr = 0x88; addr <= 0x8D; addr++) {
		iic_read(&bme, addr, &data_temp[i], 1);
		i++;
	}
	dig_T1 = (data_temp[1] << 8) | data_temp[0];
	dig_T2 = (data_temp[3] << 8) | data_temp[2];
	dig_T3 = (data_temp[5] << 8) | data_temp[4];
}

void bme_280_trim_param_press() {
	u8 data_press[18] = {0};
	u8 i = 0;
	for(int addr = 0x8E; addr <= 0x9F; addr++) {
		iic_read(&bme, addr, &data_press[i], 1);
		i++;
	}
	dig_P1 = (data_press[1] << 8) | data_press[0];
	dig_P2 = (data_press[3] << 8) | data_press[2];
	dig_P3 = (data_press[5]<< 8) | data_press[4];
	dig_P4 = (data_press[7]<< 8) | data_press[6];
	dig_P5 = (data_press[9]<< 8) | data_press[8];
	dig_P6 = (data_press[12]<< 8) | data_press[10];
	dig_P7 = (data_press[13]<< 8) | data_press[12];
	dig_P8 = (data_press[15]<< 8) | data_press[14];
	dig_P9 = (data_press[17]<< 8) | data_press[16];
}

void bme_280_trim_param_hum() {
	u8 data_hum[8] = {0};
	u8 i = 0;
	int addr = 0xA1;
	iic_read(&bme, addr, &data_hum[i], 1);
	i++;
	for(addr = 0xE1; addr <= 0xE7; addr++) {
		iic_read(&bme, addr, &data_hum[i], 1);
		i++;
	}
	dig_H1 = data_hum[0];
	dig_H2 = (data_hum[2]<< 8) | data_hum[1];
	dig_H3 = data_hum[3];
	dig_H4 = (data_hum[4]<< 4) | (0x0F & data_hum[5]);
	dig_H5 = (data_hum[6]<< 4) | ((0xF0 & data_hum[5]) >> 4);
	dig_H6 = data_hum[7];
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

BME280_U32_t bme_280_get_corrected_hum() {
	BME280_S32_t adc_H = bme_280_read_raw_hum();
    BME280_U32_t hum = bme_280_compensate_H_int32(adc_H);
    return hum;
}

float bme_280_get_hum() {
    float hum = (float)bme_280_get_corrected_hum();
    hum = hum/1024.0;
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

BME280_S32_t bme_280_get_corrected_temp() {
	BME280_S32_t adc_T = bme_280_read_raw_temp();
    BME280_S32_t temp = bme_280_compensate_T_int32(adc_T);
    return temp;
}

float bme_280_get_temp_c() {
	BME280_S32_t temp = bme_280_get_corrected_temp();
    float temp_c = ((float)temp)/100.0;
    return temp_c;
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

BME280_U32_t bme_280_get_corrected_press() {
	BME280_U32_t adc_P = bme_280_read_raw_press();
    BME280_U32_t press = bme_280_compensate_p_int64(adc_P);
    return press;
}

float bme_280_get_press_Pa() {
    float press_hPa = ((float)bme_280_get_corrected_press()/256.0);
    return press_hPa;
}

float bme_280_get_press_hPa() {
    float press_hPa = bme_280_get_press_Pa()/100.0;
    return press_hPa;
}

float bme_280_calculate_altitude_cm() {
	double exp = 1.0/5.255;
	double current_pressure_hPa = (double)bme_280_get_press_hPa();
	double p_po = current_pressure_hPa/sea_level_press_hPa;
	double pow_result = pow(p_po, exp);
	double altitude_barometer_m = 44330*(1-pow_result);
	double altitude_barometer_cm = altitude_barometer_m * 100;
	return (float)altitude_barometer_cm;
}

float bme_280_get_altitude() {
	float altitude_barometer_cm = bme_280_calculate_altitude_cm();
	float relavent_altitude = altitude_barometer_cm - altitude_barometer_start_up;
	return relavent_altitude;
}


void bme_280_alt_calibration() {
	for(int rate_calibration__number = 0; rate_calibration__number < 2000; rate_calibration__number++) {
		bme_280_calculate_altitude_cm();
		altitude_barometer_start_up+=bme_280_calculate_altitude_cm();
		usleep(1000*1);
	}
	altitude_barometer_start_up/=2000;
	usleep(1000*1000);
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

	v_x1_u32r = (t_fine -((BME280_S32_t)76800));
	v_x1_u32r = (((((adc_H << 14) -(((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) * v_x1_u32r)) +
		((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r *
		((BME280_S32_t)dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
		((BME280_S32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r -(((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
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

