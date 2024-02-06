#ifndef BME_280   /* prevent circular inclusions */
#define BME_280

#include "xiic.h"

// Default I2C address for the BME_280
#define BME_280_ADDRESS                        	0x76

#define BME_280_HUM_LSB                   		0xFE
#define BME_280_HUM_MSB                   		0xFD

#define BME_280_TEMP_XLSB						0xFC
#define BME_280_TEMP_LSB						0xFB
#define BME_280_TEMP_MSB						0xFA

#define BME_280_PRESS_XLSB						0xF9
#define BME_280_PRESS_LSB						0xF8
#define BME_280_PRESS_MSB						0xF7

#define BME_280_CONFIG							0xF5

#define BME_280_CTRL_MEAS						0xF4
#define BME_280_SLEEP_MODE						0x0
#define BME_280_FORCED_MODE_1					0x1
#define BME_280_FORCED_MODE_2					0x2
#define BME_280_NORMAL_MODE						0x3
#define BME_280_OSRS_P0							(0x0 << 2)
#define BME_280_OSRS_P1							(0x1 << 2)
#define BME_280_OSRS_P2							(0x2 << 2)
#define BME_280_OSRS_P4							(0x3 << 2)
#define BME_280_OSRS_P8							(0x4 << 2)
#define BME_280_OSRS_P16						(0x5 << 2)
#define BME_280_OSRS_T0							(0x0 << 5)
#define BME_280_OSRS_T1							(0x1 << 5)
#define BME_280_OSRS_T2							(0x2 << 5)
#define BME_280_OSRS_T4							(0x3 << 5)
#define BME_280_OSRS_T8							(0x4 << 5)
#define BME_280_OSRS_T16						(0x5 << 5)

#define BME_280_STATUS							0xF3

#define BME_280_CTRL_HUM						0xF2
#define BME_280_OSRS_H0							0x0
#define BME_280_OSRS_H1							0x1
#define BME_280_OSRS_H2							0x2
#define BME_280_OSRS_H4							0x3
#define BME_280_OSRS_H8							0x4
#define BME_280_OSRS_H16						0x5

#define BME_280_RESET							0xE0
#define BME_280_ID								0xD0

void iic_bme_init();
void bme_280_setup();
void bme_280_reset(u8 reset);
void bme_280_read_id();
void bme_280_ctrl_hum();
void bme_280_ctrl_meas();
void bme_280_read_hum();
void bme_280_read_temp();
void bme_280_read_press();

#endif  /* end of protection macro */
