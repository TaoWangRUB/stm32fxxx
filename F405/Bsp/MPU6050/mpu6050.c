/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
#include "i2c.h"
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 		0x75
#define PWR_MGMT_1_REG		0x6B
#define SMPLRT_DIV_REG		0x19
#define ACCEL_CONFIG_REG	0x1C
#define ACCEL_XOUT_H_REG	0x3B
#define TEMP_OUT_H_REG		0x41
#define GYRO_CONFIG_REG		0x1B
#define GYRO_XOUT_H_REG		0x43
#define DLPF_CONFIG_REG		0x1A // DLPF CONFIG register address
// Setup MPU6050
#define MPU6050_ADDR		0x69 << 1

const uint16_t i2c_timeout = 1000;
const double Accel_Z_corrector = 16384.0;

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

static void _MPU6050_Gyro_Calib(MPU6050_t *DataStruct)
{
	const uint16_t Num = 1000;
	float gyro_offset[3] = {0, 0, 0};

	for(uint16_t i = 0; i < Num; i++)
	{
		MPU6050_Read_Gyro(&hi2c1, DataStruct);
		for(uint8_t i = 0; i < 3; ++i)
			gyro_offset[i] += DataStruct->gyro_raw[i];
		HAL_Delay(2);
	}
	for(uint8_t i = 0; i < 3; ++i)
		DataStruct->gyro_offset[i] = gyro_offset[i] / Num;
	printf("%.5f %.5f %.5f\r\n", DataStruct->gyro_offset[0]/65.5, DataStruct->gyro_offset[1]/65.5, DataStruct->gyro_offset[2]/65.5);
	return;
}

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);
    /*if (ret == HAL_OK) {
        printf("MPU6050 WHO_AM_I: 0x%02X\r\n", check);  // Should print 0x68
    } else {
        printf("I2C Read Error: %d\r\n", ret);
    }*/
    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x09;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set Digital Low Pass Filter
        Data = 0x05;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, DLPF_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=1 -> � 500 �/s
        Data = 0x08;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // calibrate gyro
        _MPU6050_Gyro_Calib(DataStruct);
        printf("MPU6050 done gyro calib\r\n");
        return 0;
    }

    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->acce_raw[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->acce_raw[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->acce_raw[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    for(uint8_t i = 0; i < 3; ++i)
    	DataStruct->acce[i] = DataStruct->acce_raw[i] / 16384.0;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->gyro_raw[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->gyro_raw[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->gyro_raw[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 1. So I am dividing by 65.5
         for more details check GYRO_CONFIG Register              ****/

    for(uint8_t i = 0; i < 3; ++i)
    	DataStruct->gyro[i] = DataStruct->gyro_raw[i] / 65.5;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;
    DataStruct->is_reading = 1u;
    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    // Kalman angle solve
    MPU6050_Process_Data(DataStruct);

}

void MPU6050_Read_DMA(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	DataStruct->is_reading = 1u;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, DataStruct->mpu6050_raw, 14);
	//printf("DMA read state %d \r\n", (int)ret);
}

void MPU6050_Process_Data(MPU6050_t *DataStruct)
{
	uint8_t *raw = DataStruct->mpu6050_raw;
	DataStruct->acce_raw[0] = (int16_t)(raw[0] << 8 | raw[1]);
	DataStruct->acce_raw[1] = (int16_t)(raw[2] << 8 | raw[3]);
	DataStruct->acce_raw[2] = (int16_t)(raw[4] << 8 | raw[5]);
	int16_t temp = (int16_t)(raw[6] << 8 | raw[7]);
	DataStruct->gyro_raw[0] = (int16_t)(raw[8] << 8 | raw[9]);
	DataStruct->gyro_raw[1] = (int16_t)(raw[10] << 8 | raw[11]);
	DataStruct->gyro_raw[2] = (int16_t)(raw[12] << 8 | raw[13]);

	for(uint8_t i = 0; i < 3; ++i)
		DataStruct->acce[i] = DataStruct->acce_raw[i] / 16384.0;

	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	for(uint8_t i = 0; i < 3; ++i)
			DataStruct->gyro[i] = (DataStruct->gyro_raw[i] - DataStruct->gyro_offset[i]) / 65.5;

	DataStruct->is_reading = 0u;

}
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
