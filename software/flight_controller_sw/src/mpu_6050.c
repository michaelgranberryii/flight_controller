/**
 * @file axi_iic_imu.h
 * @brief Source code for the MPU_6050 driver.
 *
 * This file contains the function definitions for the MPU_6050 driver.
 * It interfaces with the MPU-6050 6-DoF Accelerometer and Gyroscope Sensor module, which uses the I2C communication protocol.
 *  - Product Link: https://www.adafruit.com/product/3886
 *  - Datasheet: http://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *  - Register Map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * The MPU-6050 module uses the following SPI configuration:
 *  - SCL Frequency: 1 MHz
 *  - MSB First
 *
 * The following connections must be made:
 *  - MPU-6050 VIN      <-->  MSP432 LaunchPad Pin VCC (3.3V)
 *  - MPU-6050 3Vo      <-->  Not Connected
 *  - MPU-6050 GND      <-->  MSP432 LaunchPad Pin GND
 *  - MPU-6050 SCL      <-->  MSP432 LaunchPad Pin P6.5 (SCL)
 *  - MPU-6050 SDA      <-->  MSP432 LaunchPad Pin P6.4 (SDA)
 *  - MPU-6050 INT      <-->  Not Connected
 *
 * For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Michael Granberry
 *
 */

#include <stdio.h>
#include "platform.h"
#include "xiic.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xparameters.h"
#include "mpu_6050.h"
#include "iic.h"
#include "sleep.h"


IIC mpu_6050;

int16_t* raw_acceleration_buffer;
float* acceleration_buffer;
float* acceleration_cal;

int16_t* raw_gyroscope_buffer;
float* gyroscope_buffer;

void mpu_calibration() {
	for (int i = 0; i < 2000; i++) {
		acceleration_buffer = mpu_6050_get_adjusted_xyz_acceleration();
		acceleration_cal[0] += acceleration_buffer[0];
		acceleration_cal[1] += acceleration_buffer[1];
		acceleration_cal[2] += acceleration_buffer[2];
//		printf("(in Accelerometer) X: %f  Y: %f  Z: %f\n\r", acceleration_cal[0], acceleration_cal[1], acceleration_cal[2]);
		usleep(1000);
	}
	acceleration_cal[0] /= 2000;
	acceleration_cal[1] /= 2000;
	acceleration_cal[2] /= 2000;

	printf("(out Accelerometer) X: %f  Y: %f  Z: %f\n\r", acceleration_cal[0], acceleration_cal[1], acceleration_cal[2]);
}

void mpu_6050_print_results() {
//	u8 accelerometer_range = MPU_6050_Get_Accelerometer_Range();
//	int accelerometer_scale = MPU_6050_Get_Accelerometer_Scale(accelerometer_range);
//	printf("Accelerometer Scale: %d\n\r", accelerometer_scale);

//	uint8_t gyroscope_range = MPU_6050_Get_Gyroscope_Range();
//	float gyroscope_scale = MPU_6050_Get_Gyroscope_Scale(gyroscope_range);
//	printf("Gyroscope Scale: %f\n\r", gyroscope_scale);


    acceleration_buffer = mpu_6050_get_adjusted_xyz_acceleration();
    printf("(Accelerometer) X: %0.2f  Y: %0.2f  Z: %0.2f\t", acceleration_buffer[0] - acceleration_cal[0], acceleration_buffer[1] - acceleration_cal[1], acceleration_buffer[2] - acceleration_cal[2]);
//    printf("%0.2f\n\r", acceleration_buffer[1]);

    gyroscope_buffer = mpu_6050_get_adjusted_xyz_gyroscope();
    printf("(Gyroscope) X: %0.2f  Y: %0.2f  Z: %0.2f\n\r", gyroscope_buffer[0], gyroscope_buffer[1], gyroscope_buffer[2]);
//    printf("%0.2f\n\r", gyroscope_buffer[0]);
//    printf("%0.2f\n\r", gyroscope_buffer[1]);
    usleep(1000*50);
}

void mpu_6050_init() {
	mpu_6050.chipAddr = MPU_6050_ADDRESS;
	iic_begin(&mpu_6050, XPAR_AXI_IIC_0_DEVICE_ID, MPU_6050_ADDRESS);
	mpu_6050_setup();
}

void mpu_6050_setup() {
	// Reset the MPU-6050 Accelerometer and Gyroscope sensor
	mpu_6050_reset();

	// Set CLKSEL config to 0x01: PLL with X axis gyroscope reference
	mpu_6050_set_clock_source(CLKSEL_PLL_X_AXIS_GYRO_REFERENCE);

    // Set sample rate value to default
    mpu_6050_set_sample_rate_divider(0x00);

    // Set Digital Low-Pass Filter Bandwidth to 21 Hz
    mpu_6050_set_dlpf_bandwidth(MPU_6050_DLPF_CFG_BANDWIDTH_10_HZ);

    // Set accelerometer range to +/- 8g
    mpu_6050_set_accelerometer_range(MPU_6050_ACCEL_RANGE_8_G);

    // Set gyroscope range to +/- 500 degrees/second
    mpu_6050_set_gyroscope_range(MPU_6050_GYRO_RANGE_500_DEG);

}

void mpu_6050_reset()
{
	u8 buf;
	buf = MPU_REG_SET_BIT_7;
	iic_write(&mpu_6050, MPU_6050_PWR_MGMT_1, &buf, 1);
	usleep(ONE_HUNDRED_mSec);

	buf = MPU_REG_SET_BIT_2 | MPU_REG_SET_BIT_1 | MPU_REG_SET_BIT_0;
	iic_write(&mpu_6050, MPU_6050_SIGNAL_PATH_RESET, &buf, 1);
	usleep(ONE_HUNDRED_mSec);
}

void mpu_6050_set_clock_source(u8 clock_source_select)
{
	iic_write(&mpu_6050, MPU_6050_PWR_MGMT_1, &clock_source_select, 1);
    usleep(ONE_HUNDRED_mSec);
}

void mpu_6050_set_sample_rate_divider(uint8_t sample_rate_divider)
{
	iic_write(&mpu_6050, MPU_6050_SMPRT_DIV, &sample_rate_divider, 1);
}

void mpu_6050_set_dlpf_bandwidth(uint8_t dlpf_cfg)
{
	iic_write(&mpu_6050, MPU_6050_CONFIG, &dlpf_cfg, 1);
}

void mpu_6050_set_accelerometer_range(uint8_t accelerometer_range)
{
	iic_write(&mpu_6050, MPU_6050_ACCEL_CONFIG, &accelerometer_range, 1);
}

void mpu_6050_set_gyroscope_range(uint8_t gyroscope_range)
{
	iic_write(&mpu_6050, MPU_6050_GYRO_CONFIG, &gyroscope_range, 1);
}


u8 mpu_6050_get_accelerometer_range()
{
    u8 MPU_6050_Accelerometer_Range;
    iic_read(&mpu_6050, MPU_6050_ACCEL_CONFIG, &MPU_6050_Accelerometer_Range, 1);
    // Bit 4 and Bit 3 of the ACCEL_CONFIG register represents AFS_SEL[1:0]
    MPU_6050_Accelerometer_Range = (MPU_6050_Accelerometer_Range & (MPU_REG_SET_BIT_4 | MPU_REG_SET_BIT_3));

    return MPU_6050_Accelerometer_Range;
}

int mpu_6050_get_accelerometer_scale(uint8_t accelerometer_range)
{
    int accelerometer_scale;

    switch(accelerometer_range)
    {
        case MPU_6050_ACCEL_RANGE_2_G:
        {
            accelerometer_scale = 16384;
            break;
        }

        case MPU_6050_ACCEL_RANGE_4_G:
        {
            accelerometer_scale = 8192;
            break;
        }

        case MPU_6050_ACCEL_RANGE_8_G:
        {
            accelerometer_scale = 4096;
            break;
        }

        case MPU_6050_ACCEL_RANGE_16_G:
        {
            accelerometer_scale = 2048;
            break;
        }
    }
    return accelerometer_scale;
}

uint8_t mpu_6050_get_accel_x_high_byte()
{
    uint8_t Accel_X_High_Byte;
    iic_read(&mpu_6050, MPU_6050_ACCEL_XOUT_H, &Accel_X_High_Byte, 1);
    return Accel_X_High_Byte;
}

uint8_t mpu_6050_get_accel_x_low_byte()
{
    uint8_t Accel_X_Low_Byte;
    iic_read(&mpu_6050, MPU_6050_ACCEL_XOUT_L, &Accel_X_Low_Byte, 1);
    return Accel_X_Low_Byte;
}

uint8_t mpu_6050_get_accel_y_high_byte()
{
    uint8_t Accel_Y_High_Byte;
    iic_read(&mpu_6050, MPU_6050_ACCEL_YOUT_H, &Accel_Y_High_Byte, 1);
    return Accel_Y_High_Byte;
}

uint8_t mpu_6050_get_accel_y_low_byte()
{
    uint8_t Accel_Y_Low_Byte;
    iic_read(&mpu_6050, MPU_6050_ACCEL_YOUT_L, &Accel_Y_Low_Byte, 1);
    return Accel_Y_Low_Byte;
}

uint8_t mpu_6050_get_accel_z_high_byte()
{
    uint8_t Accel_Z_High_Byte;
    iic_read(&mpu_6050, MPU_6050_ACCEL_ZOUT_H, &Accel_Z_High_Byte, 1);
    return Accel_Z_High_Byte;
}

uint8_t mpu_6050_get_accel_z_low_byte()
{
    uint8_t Accel_Z_Low_Byte;
    iic_read(&mpu_6050, MPU_6050_ACCEL_ZOUT_L, &Accel_Z_Low_Byte, 1);
    return Accel_Z_Low_Byte;
}

int16_t* mpu_6050_get_raw_xyz_acceleration()
{
    static int16_t raw_acceleration_buffer[3];

    uint8_t Raw_Acceleration_X_High_Byte = mpu_6050_get_accel_x_high_byte();
    uint8_t Raw_Acceleration_X_Low_Byte = mpu_6050_get_accel_x_low_byte();
    int16_t Raw_Acceleration_X = (Raw_Acceleration_X_High_Byte << 8) | Raw_Acceleration_X_Low_Byte;

    uint8_t Raw_Acceleration_Y_High_Byte = mpu_6050_get_accel_y_high_byte();
    uint8_t Raw_Acceleration_Y_Low_Byte = mpu_6050_get_accel_y_low_byte();
    int16_t Raw_Acceleration_Y = (Raw_Acceleration_Y_High_Byte << 8) | Raw_Acceleration_Y_Low_Byte;

    uint8_t Raw_Acceleration_Z_High_Byte = mpu_6050_get_accel_z_high_byte();
    uint8_t Raw_Acceleration_Z_Low_Byte = mpu_6050_get_accel_z_low_byte();
    int16_t Raw_Acceleration_Z = (Raw_Acceleration_Z_High_Byte << 8) | Raw_Acceleration_Z_Low_Byte;

    raw_acceleration_buffer[0] = Raw_Acceleration_X;
    raw_acceleration_buffer[1] = Raw_Acceleration_Y;
    raw_acceleration_buffer[2] = Raw_Acceleration_Z;

    return raw_acceleration_buffer;
}

float* mpu_6050_get_adjusted_xyz_acceleration()
{
    int16_t* raw_acceleration_buffer;
    static float acceleration_buffer[3];

    uint8_t accelerometer_range = mpu_6050_get_accelerometer_range();
    int accelerometer_scale = mpu_6050_get_accelerometer_scale(accelerometer_range);

    raw_acceleration_buffer = mpu_6050_get_raw_xyz_acceleration();

    float Acceleration_X = ((float)(raw_acceleration_buffer[0])) / accelerometer_scale;
    float Acceleration_Y = ((float)(raw_acceleration_buffer[1])) / accelerometer_scale;
    float Acceleration_Z = ((float)(raw_acceleration_buffer[2])) / accelerometer_scale;

    acceleration_buffer[0] = Acceleration_X;
    acceleration_buffer[1] = Acceleration_Y;
    acceleration_buffer[2] = Acceleration_Z;

    return acceleration_buffer;
}

uint8_t mpu_6050_get_gyroscope_range()
{
    uint8_t MPU_6050_Gyroscope_Range;
    iic_read(&mpu_6050, MPU_6050_GYRO_CONFIG, &MPU_6050_Gyroscope_Range, 1);

    // Bit 4 and Bit 3 of the GYRO_CONFIG register represents FS_SEL[1:0]
    MPU_6050_Gyroscope_Range = (MPU_6050_Gyroscope_Range & (MPU_REG_SET_BIT_4 | MPU_REG_SET_BIT_3));

    return MPU_6050_Gyroscope_Range;
}

float mpu_6050_get_gyroscope_scale(uint8_t gyroscope_range)
{
    float gyroscope_scale;

    switch(gyroscope_range)
    {
        case MPU_6050_GYRO_RANGE_250_DEG:
        {
            gyroscope_scale = 131.0;
            break;
        }

        case MPU_6050_GYRO_RANGE_500_DEG:
        {
            gyroscope_scale = 65.5;
            break;
        }

        case MPU_6050_GYRO_RANGE_1000_DEG:
        {
            gyroscope_scale = 32.8;
            break;
        }

        case MPU_6050_GYRO_RANGE_2000_DEG:
        {
            gyroscope_scale = 16.4;
            break;
        }
    }
    return gyroscope_scale;
}

uint8_t mpu_6050_get_gyro_x_high_byte()
{
    uint8_t gyro_x_high_byte;
    iic_read(&mpu_6050, MPU_6050_GYRO_XOUT_H, &gyro_x_high_byte, 1);
    return gyro_x_high_byte;
}

uint8_t mpu_6050_get_gyro_x_low_byte()
{
    uint8_t gyro_x_low_byte;
    iic_read(&mpu_6050, MPU_6050_GYRO_XOUT_L, &gyro_x_low_byte, 1);
    return gyro_x_low_byte;
}

uint8_t mpu_6050_get_gyro_y_high_byte()
{
    uint8_t gyro_y_high_byte;
    iic_read(&mpu_6050, MPU_6050_GYRO_YOUT_H, &gyro_y_high_byte, 1);
    return gyro_y_high_byte;
}

uint8_t mpu_6050_get_gyro_y_low_byte()
{
    uint8_t gyro_y_low_byte;
    iic_read(&mpu_6050, MPU_6050_GYRO_YOUT_L, &gyro_y_low_byte, 1);
    return gyro_y_low_byte;
}

uint8_t mpu_6050_get_gyro_z_high_byte()
{
    uint8_t gyro_z_high_byte;
    iic_read(&mpu_6050, MPU_6050_GYRO_ZOUT_H, &gyro_z_high_byte, 1);
    return gyro_z_high_byte;
}

uint8_t mpu_6050_get_gyro_z_low_byte()
{
    uint8_t gyro_z_low_byte;
    iic_read(&mpu_6050, MPU_6050_GYRO_ZOUT_L, &gyro_z_low_byte, 1);
    return gyro_z_low_byte;
}


int16_t* mpu_6050_get_raw_xyz_gyroscope()
{
    static int16_t raw_gyroscope_buffer[3];

    uint8_t Raw_Gyroscope_X_High_Byte = mpu_6050_get_gyro_x_high_byte();
    uint8_t Raw_Gyroscope_X_Low_Byte = mpu_6050_get_gyro_x_low_byte();
    int16_t Raw_Gyroscope_X = (Raw_Gyroscope_X_High_Byte << 8) | Raw_Gyroscope_X_Low_Byte;

    uint8_t Raw_Gyroscope_Y_High_Byte = mpu_6050_get_gyro_y_high_byte();
    uint8_t Raw_Gyroscope_Y_Low_Byte = mpu_6050_get_gyro_y_low_byte();
    int16_t Raw_Gyroscope_Y = (Raw_Gyroscope_Y_High_Byte << 8) | Raw_Gyroscope_Y_Low_Byte;

    uint8_t Raw_Gyroscope_Z_High_Byte = mpu_6050_get_gyro_z_high_byte();
    uint8_t Raw_Gyroscope_Z_Low_Byte = mpu_6050_get_gyro_z_low_byte();
    int16_t Raw_Gyroscope_Z = (Raw_Gyroscope_Z_High_Byte << 8) | Raw_Gyroscope_Z_Low_Byte;

    raw_gyroscope_buffer[0] = Raw_Gyroscope_X;
    raw_gyroscope_buffer[1] = Raw_Gyroscope_Y;
    raw_gyroscope_buffer[2] = Raw_Gyroscope_Z;

    return raw_gyroscope_buffer;
}

float* mpu_6050_get_adjusted_xyz_gyroscope()
{
    int16_t* raw_gyroscope_buffer;
    static float gyroscope_buffer[3];

    uint8_t gyroscope_range = mpu_6050_get_gyroscope_range();
    int gyroscope_scale = mpu_6050_get_gyroscope_scale(gyroscope_range);

    raw_gyroscope_buffer = mpu_6050_get_raw_xyz_gyroscope();

    float Gyroscope_X = ((float)(raw_gyroscope_buffer[0])) / gyroscope_scale;
    float Gyroscope_Y = ((float)(raw_gyroscope_buffer[1])) / gyroscope_scale;
    float Gyroscope_Z = ((float)(raw_gyroscope_buffer[2])) / gyroscope_scale;

    gyroscope_buffer[0] = Gyroscope_X;
    gyroscope_buffer[1] = Gyroscope_Y;
    gyroscope_buffer[2] = Gyroscope_Z;

    return gyroscope_buffer;
}


