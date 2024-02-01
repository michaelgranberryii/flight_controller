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
#include "axi_iic_imu.h"
#include "sleep.h"

XIic_Config *iic_config;
XIic iic_imu;

const u8 chipAddr = 0x68;
u8 currentRegister;
u8 recvbytes;
u8 *recv;

int16_t* raw_acceleration_buffer;
float* acceleration_buffer;
float* acceleration_cal;

int16_t* raw_gyroscope_buffer;
float* gyroscope_buffer;

void MPU_Calibration() {
	for (int i = 0; i < 2000; i++) {
		acceleration_buffer = MPU_6050_Get_Adjusted_XYZ_Acceleration();
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

void MPU_Print_Results() {
//	u8 accelerometer_range = MPU_6050_Get_Accelerometer_Range();
//	int accelerometer_scale = MPU_6050_Get_Accelerometer_Scale(accelerometer_range);
//	printf("Accelerometer Scale: %d\n\r", accelerometer_scale);

//	uint8_t gyroscope_range = MPU_6050_Get_Gyroscope_Range();
//	float gyroscope_scale = MPU_6050_Get_Gyroscope_Scale(gyroscope_range);
//	printf("Gyroscope Scale: %f\n\r", gyroscope_scale);


//    acceleration_buffer = MPU_6050_Get_Adjusted_XYZ_Acceleration();
//    printf("(Accelerometer) X: %0.2f  Y: %0.2f  Z: %0.2f\t", acceleration_buffer[0] - (-0.022839), acceleration_buffer[1] - (0.018668 ), acceleration_buffer[2] - (1.037594));
//    printf("%0.2f\n\r", acceleration_buffer[1]);

    gyroscope_buffer = MPU_6050_Get_Adjusted_XYZ_Gyroscope();
//    printf("(Gyroscope) X: %0.2f  Y: %0.2f  Z: %0.2f\n\r", gyroscope_buffer[0], gyroscope_buffer[1], gyroscope_buffer[2]);
    printf("%0.2f\n\r", gyroscope_buffer[0]);
//    printf("%0.2f\n\r", gyroscope_buffer[1]);
    usleep(1000*50);
}

void iic_imu_init() {
	int status = XIic_Initialize(&iic_imu, XPAR_AXI_IIC_0_DEVICE_ID);
	if(status == XST_SUCCESS)
		xil_printf("IIC IMU Init Successful\n\r");
	else
		xil_printf("IIC IMU Init Failed\n\r");
	 status = XIic_SelfTest(&iic_imu);
	if(status == XST_SUCCESS) {
		xil_printf("IIC IMU SELF TEST SUCCUSSFUL\n\r");
	} else {
		xil_printf("IIC IMU SELF TEST FAILED\n\r");
	}
}

void iic_imu_setup() {
	XIic_Stop(&iic_imu);
	XIic_SetAddress(&iic_imu, XII_ADDR_TO_SEND_TYPE, chipAddr);
//	u32 iic_options = XIic_GetOptions(&iic_imu);
//	iic_options |= XII_GENERAL_CALL_OPTION;

	// Reset the MPU-6050 Accelerometer and Gyroscope sensor
	MPU_6050_Reset();

	// test
	u8 buf;
	buf = 0x0d;
	MPU_WriteIIC(I2C_MST_CTRL, &buf, 1);
	MPU_ReadIIC(I2C_MST_CTRL, &buf, 1);
	// end test

	// Set CLKSEL config to 0x01: PLL with X axis gyroscope reference
	MPU_6050_Set_Clock_Source(CLKSEL_PLL_X_AXIS_GYRO_REFERENCE);

    // Set sample rate value to default
    MPU_6050_Set_Sample_Rate_Divider(0x00);

    // Set Digital Low-Pass Filter Bandwidth to 21 Hz
    MPU_6050_Set_DLPF_Bandwidth(MPU_6050_DLPF_CFG_BANDWIDTH_10_HZ);

    // Set accelerometer range to +/- 8g
    MPU_6050_Set_Accelerometer_Range(MPU_6050_ACCEL_RANGE_8_G);

    // Set gyroscope range to +/- 500 degrees/second
    MPU_6050_Set_Gyroscope_Range(MPU_6050_GYRO_RANGE_500_DEG);

}

void MPU_6050_Reset()
{
	u8 buf;
	buf = MPU_REG_SET_BIT_7;
	MPU_WriteIIC(MPU_6050_PWR_MGMT_1, &buf, 1);
	usleep(ONE_HUNDRED_mSec);

	buf = MPU_REG_SET_BIT_2 | MPU_REG_SET_BIT_1 | MPU_REG_SET_BIT_0;
	MPU_WriteIIC(MPU_6050_SIGNAL_PATH_RESET, &buf, 1);
	usleep(ONE_HUNDRED_mSec);
}

void MPU_6050_Set_Clock_Source(u8 clock_source_select)
{
	MPU_WriteIIC(MPU_6050_PWR_MGMT_1, &clock_source_select, 1);
    usleep(ONE_HUNDRED_mSec);
}

void MPU_6050_Set_Sample_Rate_Divider(uint8_t sample_rate_divider)
{
    MPU_WriteIIC(MPU_6050_SMPRT_DIV, &sample_rate_divider, 1);
}

void MPU_6050_Set_DLPF_Bandwidth(uint8_t dlpf_cfg)
{
    MPU_WriteIIC(MPU_6050_CONFIG, &dlpf_cfg, 1);
}

void MPU_6050_Set_Accelerometer_Range(uint8_t accelerometer_range)
{
    MPU_WriteIIC(MPU_6050_ACCEL_CONFIG, &accelerometer_range, 1);
}

void MPU_6050_Set_Gyroscope_Range(uint8_t gyroscope_range)
{
    MPU_WriteIIC(MPU_6050_GYRO_CONFIG, &gyroscope_range, 1);
}


u8 MPU_6050_Get_Accelerometer_Range()
{
    u8 MPU_6050_Accelerometer_Range;
    MPU_ReadIIC(MPU_6050_ACCEL_CONFIG, &MPU_6050_Accelerometer_Range, 1);
    // Bit 4 and Bit 3 of the ACCEL_CONFIG register represents AFS_SEL[1:0]
    MPU_6050_Accelerometer_Range = (MPU_6050_Accelerometer_Range & (MPU_REG_SET_BIT_4 | MPU_REG_SET_BIT_3));

    return MPU_6050_Accelerometer_Range;
}

int MPU_6050_Get_Accelerometer_Scale(uint8_t accelerometer_range)
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

uint8_t MPU_6050_Get_Accel_X_High_Byte()
{
    uint8_t Accel_X_High_Byte;
    MPU_ReadIIC(MPU_6050_ACCEL_XOUT_H, &Accel_X_High_Byte, 1);
    return Accel_X_High_Byte;
}

uint8_t MPU_6050_Get_Accel_X_Low_Byte()
{
    uint8_t Accel_X_Low_Byte;
    MPU_ReadIIC(MPU_6050_ACCEL_XOUT_L, &Accel_X_Low_Byte, 1);
    return Accel_X_Low_Byte;
}

uint8_t MPU_6050_Get_Accel_Y_High_Byte()
{
    uint8_t Accel_Y_High_Byte;
    MPU_ReadIIC(MPU_6050_ACCEL_YOUT_H, &Accel_Y_High_Byte, 1);
    return Accel_Y_High_Byte;
}

uint8_t MPU_6050_Get_Accel_Y_Low_Byte()
{
    uint8_t Accel_Y_Low_Byte;
    MPU_ReadIIC(MPU_6050_ACCEL_YOUT_L, &Accel_Y_Low_Byte, 1);
    return Accel_Y_Low_Byte;
}

uint8_t MPU_6050_Get_Accel_Z_High_Byte()
{
    uint8_t Accel_Z_High_Byte;
    MPU_ReadIIC(MPU_6050_ACCEL_ZOUT_H, &Accel_Z_High_Byte, 1);
    return Accel_Z_High_Byte;
}

uint8_t MPU_6050_Get_Accel_Z_Low_Byte()
{
    uint8_t Accel_Z_Low_Byte;
    MPU_ReadIIC(MPU_6050_ACCEL_ZOUT_L, &Accel_Z_Low_Byte, 1);
    return Accel_Z_Low_Byte;
}

int16_t* MPU_6050_Get_Raw_XYZ_Acceleration()
{
    static int16_t raw_acceleration_buffer[3];

    uint8_t Raw_Acceleration_X_High_Byte = MPU_6050_Get_Accel_X_High_Byte();
    uint8_t Raw_Acceleration_X_Low_Byte = MPU_6050_Get_Accel_X_Low_Byte();
    int16_t Raw_Acceleration_X = (Raw_Acceleration_X_High_Byte << 8) | Raw_Acceleration_X_Low_Byte;

    uint8_t Raw_Acceleration_Y_High_Byte = MPU_6050_Get_Accel_Y_High_Byte();
    uint8_t Raw_Acceleration_Y_Low_Byte = MPU_6050_Get_Accel_Y_Low_Byte();
    int16_t Raw_Acceleration_Y = (Raw_Acceleration_Y_High_Byte << 8) | Raw_Acceleration_Y_Low_Byte;

    uint8_t Raw_Acceleration_Z_High_Byte = MPU_6050_Get_Accel_Z_High_Byte();
    uint8_t Raw_Acceleration_Z_Low_Byte = MPU_6050_Get_Accel_Z_Low_Byte();
    int16_t Raw_Acceleration_Z = (Raw_Acceleration_Z_High_Byte << 8) | Raw_Acceleration_Z_Low_Byte;

    raw_acceleration_buffer[0] = Raw_Acceleration_X;
    raw_acceleration_buffer[1] = Raw_Acceleration_Y;
    raw_acceleration_buffer[2] = Raw_Acceleration_Z;

    return raw_acceleration_buffer;
}

float* MPU_6050_Get_Adjusted_XYZ_Acceleration()
{
    int16_t* raw_acceleration_buffer;
    static float acceleration_buffer[3];

    uint8_t accelerometer_range = MPU_6050_Get_Accelerometer_Range();
    int accelerometer_scale = MPU_6050_Get_Accelerometer_Scale(accelerometer_range);

    raw_acceleration_buffer = MPU_6050_Get_Raw_XYZ_Acceleration();

    float Acceleration_X = ((float)(raw_acceleration_buffer[0])) / accelerometer_scale;
    float Acceleration_Y = ((float)(raw_acceleration_buffer[1])) / accelerometer_scale;
    float Acceleration_Z = ((float)(raw_acceleration_buffer[2])) / accelerometer_scale;

    acceleration_buffer[0] = Acceleration_X;
    acceleration_buffer[1] = Acceleration_Y;
    acceleration_buffer[2] = Acceleration_Z;

    return acceleration_buffer;
}

uint8_t MPU_6050_Get_Gyroscope_Range()
{
    uint8_t MPU_6050_Gyroscope_Range;
    MPU_ReadIIC(MPU_6050_GYRO_CONFIG, &MPU_6050_Gyroscope_Range, 1);

    // Bit 4 and Bit 3 of the GYRO_CONFIG register represents FS_SEL[1:0]
    MPU_6050_Gyroscope_Range = (MPU_6050_Gyroscope_Range & (MPU_REG_SET_BIT_4 | MPU_REG_SET_BIT_3));

    return MPU_6050_Gyroscope_Range;
}

float MPU_6050_Get_Gyroscope_Scale(uint8_t gyroscope_range)
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

uint8_t MPU_6050_Get_Gyro_X_High_Byte()
{
    uint8_t Gyro_X_High_Byte;
    MPU_ReadIIC(MPU_6050_GYRO_XOUT_H, &Gyro_X_High_Byte, 1);
    return Gyro_X_High_Byte;
}

uint8_t MPU_6050_Get_Gyro_X_Low_Byte()
{
    uint8_t Gyro_X_Low_Byte;
    MPU_ReadIIC(MPU_6050_GYRO_XOUT_L, &Gyro_X_Low_Byte, 1);
    return Gyro_X_Low_Byte;
}

uint8_t MPU_6050_Get_Gyro_Y_High_Byte()
{
    uint8_t Gyro_Y_High_Byte;
    MPU_ReadIIC(MPU_6050_GYRO_YOUT_H, &Gyro_Y_High_Byte, 1);
    return Gyro_Y_High_Byte;
}

uint8_t MPU_6050_Get_Gyro_Y_Low_Byte()
{
    uint8_t Gyro_Y_Low_Byte;
    MPU_ReadIIC(MPU_6050_GYRO_YOUT_L, &Gyro_Y_Low_Byte, 1);
    return Gyro_Y_Low_Byte;
}

uint8_t MPU_6050_Get_Gyro_Z_High_Byte()
{
    uint8_t Gyro_Z_High_Byte;
    MPU_ReadIIC(MPU_6050_GYRO_ZOUT_H, &Gyro_Z_High_Byte, 1);
    return Gyro_Z_High_Byte;
}

uint8_t MPU_6050_Get_Gyro_Z_Low_Byte()
{
    uint8_t Gyro_Z_Low_Byte;
    MPU_ReadIIC(MPU_6050_GYRO_ZOUT_L, &Gyro_Z_Low_Byte, 1);
    return Gyro_Z_Low_Byte;
}

int16_t* MPU_6050_Get_Raw_XYZ_Gyroscope()
{
    static int16_t raw_gyroscope_buffer[3];

    uint8_t Raw_Gyroscope_X_High_Byte = MPU_6050_Get_Gyro_X_High_Byte();
    uint8_t Raw_Gyroscope_X_Low_Byte = MPU_6050_Get_Gyro_X_Low_Byte();
    int16_t Raw_Gyroscope_X = (Raw_Gyroscope_X_High_Byte << 8) | Raw_Gyroscope_X_Low_Byte;

    uint8_t Raw_Gyroscope_Y_High_Byte = MPU_6050_Get_Gyro_Y_High_Byte();
    uint8_t Raw_Gyroscope_Y_Low_Byte = MPU_6050_Get_Gyro_Y_Low_Byte();
    int16_t Raw_Gyroscope_Y = (Raw_Gyroscope_Y_High_Byte << 8) | Raw_Gyroscope_Y_Low_Byte;

    uint8_t Raw_Gyroscope_Z_High_Byte = MPU_6050_Get_Gyro_Z_High_Byte();
    uint8_t Raw_Gyroscope_Z_Low_Byte = MPU_6050_Get_Gyro_Z_Low_Byte();
    int16_t Raw_Gyroscope_Z = (Raw_Gyroscope_Z_High_Byte << 8) | Raw_Gyroscope_Z_Low_Byte;

    raw_gyroscope_buffer[0] = Raw_Gyroscope_X;
    raw_gyroscope_buffer[1] = Raw_Gyroscope_Y;
    raw_gyroscope_buffer[2] = Raw_Gyroscope_Z;

    return raw_gyroscope_buffer;
}

float* MPU_6050_Get_Adjusted_XYZ_Gyroscope()
{
    int16_t* raw_gyroscope_buffer;
    static float gyroscope_buffer[3];

    uint8_t gyroscope_range = MPU_6050_Get_Gyroscope_Range();
    int gyroscope_scale = MPU_6050_Get_Gyroscope_Scale(gyroscope_range);

    raw_gyroscope_buffer = MPU_6050_Get_Raw_XYZ_Gyroscope();

    float Gyroscope_X = ((float)(raw_gyroscope_buffer[0])) / gyroscope_scale;
    float Gyroscope_Y = ((float)(raw_gyroscope_buffer[1])) / gyroscope_scale;
    float Gyroscope_Z = ((float)(raw_gyroscope_buffer[2])) / gyroscope_scale;

    gyroscope_buffer[0] = Gyroscope_X;
    gyroscope_buffer[1] = Gyroscope_Y;
    gyroscope_buffer[2] = Gyroscope_Z;

    return gyroscope_buffer;
}

void MPU_WriteIIC(u8 reg, u8 *Data, int nData) {
   u8 out[10];
   out[0] = reg;
   out[1] = *Data;
   int Status;

   if (currentRegister != reg) {
      currentRegister = reg;
   }
   Status = XIic_Start(&iic_imu);
   if (Status != XST_SUCCESS) {
      return;
   }
   XIic_Send(XPAR_AXI_IIC_0_BASEADDR, chipAddr, out, nData + 1, XIIC_STOP);

   Status = XIic_Stop(&iic_imu);
   if (Status != XST_SUCCESS) {
      return;
   }
}

void MPU_ReadIIC(u8 reg, u8 *Data, int nData) {
   int Status;
   Status = XIic_Start(&iic_imu);
   if (Status != XST_SUCCESS) {
      return;
   }
   if (currentRegister != reg) {

      XIic_Send(XPAR_AXI_IIC_0_BASEADDR, chipAddr, &reg, 1, XII_REPEATED_START_OPTION);
      currentRegister = reg;
   }
   XIic_Recv(XPAR_AXI_IIC_0_BASEADDR, chipAddr, Data, nData, XIIC_STOP);

   Status = XIic_Stop(&iic_imu);
   if (Status != XST_SUCCESS) {
      return;
   }
}


