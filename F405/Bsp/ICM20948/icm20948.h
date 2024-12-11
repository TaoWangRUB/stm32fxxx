/*
 * icm20948.h
 *
 *  Created on: Dec 5, 2024
 *      Author: taowang
 */

/* for acc and gyro */
#ifndef ICM20948_ICM20948_H_
#define ICM20948_ICM20948_H_

#include <stdint.h>

// ICM20948 I2C Address
#define ICM20948_ADDR  0x68 << 1 // Shifted for STM32 HAL

// Bank switching
#define ICM20948_REG_BANK_SEL  0x7F
#define ICM20948_USER_BANK_0   0x00
#define ICM20948_USER_BANK_1   0x10
#define ICM20948_USER_BANK_2   0x20
#define ICM20948_USER_BANK_3   0x30

// Bank 0 Registers
#define ICM20948_PWR_MGMT_1    0x06
#define ICM20948_PWR_MGMT_2    0x07
#define ICM20948_WHO_AM_I      0x00
#define ICM20948_ACCEL_XOUT_H  0x2D
#define ICM20948_GYRO_XOUT_H   0x33

// Bank 2 Registers
#define ICM20948_GYRO_SMPLRT_DIV 	0x00
#define ICM20948_GYRO_CONFIG	0x01
#define REG_VAL_BIT_GYRO_DLPCFG_2   0x10 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_4   0x20 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_6   0x30 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_FS_250DPS  0x00 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_500DPS  0x02 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_DLPF       0x01 /* bit[0]   */

#define ICM20948_ACCEL_SMPLRT_DIV	0x11
#define ICM20948_ACCEL_CONFIG	0x14
#define REG_VAL_BIT_ACCEL_DLPCFG_2  0x10 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_4  0x20 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_6  0x30 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_FS_2g     0x00 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_4g     0x02 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_8g     0x04 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_16g    0x06 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_DLPF      0x01 /* bit[0]   */

// Magnetometer Registers
#define ICM20948_MAG_ADDR      0x0C << 1 // Magnetometer I2C address
#define AK09916_WIA2           0x01
#define AK09916_CNTL2          0x31
#define AK09916_DATA_READY     0x10
#define AK09916_HXL            0x11

// Magnetometer Control Bits
#define AK09916_CNTL2_MODE_CONT 0x08

#define REG_ADD_USER_CTRL          0x03  // USER_CTRL register
#define REG_ADD_REG_BANK_SEL       0x7F  // BANK_SEL register
#define REG_ADD_I2C_SLV0_ADDR      0x03  // I2C_SLV0_ADDR register
#define REG_ADD_I2C_SLV0_REG       0x04  // I2C_SLV0_REG register
#define REG_ADD_I2C_SLV0_CTRL      0x05  // I2C_SLV0_CTRL register
#define REG_ADD_I2C_SLV0_DO        0x06  // I2C_SLV0_DO register
#define REG_ADD_EXT_SENS_DATA_00   0x3B  // External Sensor Data register 00

// ICM20948 Bit Masks and Values
#define REG_VAL_BIT_I2C_MST_EN     0x20  // I2C_MST_EN bit for USER_CTRL
#define REG_VAL_BIT_SLV0_EN        0x80  // Enable Slave 0
#define REG_VAL_REG_BANK_0         ICM20948_USER_BANK_0
#define REG_VAL_REG_BANK_3         ICM20948_USER_BANK_3

#define REG_ADD_INT_PIN_CFG    		0x0F  // INT_PIN_CFG register address
#define REG_VAL_BIT_I2C_BYPASS_EN 	0x02 // Enable I2C bypass mode
// ICM20948 structure
typedef struct icm20948_type_def
{
	uint8_t acc_gyro_buff[14];	 // 6 bytes for accel, 6 for gyro, 2 for temp
	uint8_t magn_buff[6];		 // 6 bytes for magnetic
	int16_t acce_raw[3];
	int16_t gyro_raw[3];
	float gyro_offset[3];
	float acce[3];
	float gyro[3];
	float mage[3];
	float angle[3];
    uint8_t is_reading;
} ICM20948_t;

// Initialization function
uint8_t ICM20948_Init(ICM20948_t* imu_data);

void ICM20948_ReadDMA(ICM20948_t* imu_data);

void ICM20948_Process_Gyro_data(ICM20948_t* imu_data);

uint8_t Magnetometer_Init(void);

void Magnetometer_ReadDMA(ICM20948_t* imu_data);

void ICM20948_Process_Mage_data(ICM20948_t* imu_data);

#endif /* ICM20948_ICM20948_H_ */
