/*
 * icm20948.c
 *
 *  Created on: Dec 5, 2024
 *      Author: taowang
 */


#include "icm20948.h"
#include "i2c.h"

void _ICM20948_Read_Gyro(ICM20948_t* imu_data)
{
	uint8_t * buff = imu_data->acc_gyro_buff;
	HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDR, ICM20948_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buff, 14, 1000);
	imu_data->acce_raw[0] = (int16_t)((buff[0] << 8) | buff[1]);
	imu_data->acce_raw[1] = (int16_t)((buff[2] << 8) | buff[3]);
	imu_data->acce_raw[2] = (int16_t)((buff[4] << 8) | buff[5]);

	// Read Gyroscope Data
	imu_data->gyro_raw[0] = (int16_t)((buff[8] << 8) | buff[9]);
	imu_data->gyro_raw[1] = (int16_t)((buff[10] << 8) | buff[11]);
	imu_data->gyro_raw[2] = (int16_t)((buff[12] << 8) | buff[13]);
	return;
}

static void _ICM20948_Gyro_Calib(ICM20948_t* imu_data)
{
	const uint16_t Num = 2000;
	for(uint16_t i = 0; i < Num; i ++)
	{
		_ICM20948_Read_Gyro(imu_data);
		for(uint8_t i = 0; i < 3; ++i)
			imu_data->gyro_offset[i] += imu_data->gyro_raw[i];
		HAL_Delay(2);
	}
	for(uint8_t i = 0; i < 3; ++i)
		imu_data->gyro_offset[i] /= Num;
	printf("%.5f %.5f %.5f\r\n", imu_data->gyro_offset[0]/65.5, imu_data->gyro_offset[1]/65.5, imu_data->gyro_offset[2]/65.5);
	return;
}

// Function to write to a register
HAL_StatusTypeDef ICM20948_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);
}

// Function to read from a register
HAL_StatusTypeDef ICM20948_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(hi2c, ICM20948_ADDR, reg, 1, data, 1, HAL_MAX_DELAY);
}

// Function to switch bank
HAL_StatusTypeDef ICM20948_SwitchBank(I2C_HandleTypeDef *hi2c, uint8_t bank) {
    return ICM20948_WriteRegister(hi2c, ICM20948_REG_BANK_SEL, bank);
}

// Initialization function
HAL_StatusTypeDef ICM20948_Init(ICM20948_t* imu_data) {
    uint8_t data;

    // Reset ICM20948
    ICM20948_WriteRegister(&hi2c1, ICM20948_REG_BANK_SEL, ICM20948_USER_BANK_0);
    ICM20948_WriteRegister(&hi2c1, ICM20948_PWR_MGMT_1, 0x80); // Reset device
    HAL_Delay(10);

    // Wake up ICM20948
    ICM20948_WriteRegister(&hi2c1, ICM20948_PWR_MGMT_1, 0x01); // Enable clock
    //ICM20948_WriteRegister(&hi2c1, ICM20948_PWR_MGMT_2, 0x00); // Enable all
    //ICM20948_SwitchBank(&hi2c1, ICM20948_USER_BANK_2);
    ICM20948_WriteRegister(&hi2c1, ICM20948_REG_BANK_SEL, ICM20948_USER_BANK_2);

    // Configure gyroscope
    ICM20948_WriteRegister(&hi2c1, ICM20948_GYRO_SMPLRT_DIV, 0x09);
    ICM20948_WriteRegister(&hi2c1, ICM20948_GYRO_CONFIG,
                      REG_VAL_BIT_GYRO_DLPCFG_2 | REG_VAL_BIT_GYRO_FS_500DPS | REG_VAL_BIT_GYRO_DLPF);
    // Configure accelerometer
    ICM20948_WriteRegister(&hi2c1, ICM20948_ACCEL_SMPLRT_DIV,  0x09);
    ICM20948_WriteRegister(&hi2c1, ICM20948_ACCEL_CONFIG,
                      REG_VAL_BIT_ACCEL_DLPCFG_2 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);

    /* user bank 0 register */
    ICM20948_WriteRegister(&hi2c1, ICM20948_REG_BANK_SEL, ICM20948_USER_BANK_0);
    HAL_Delay(10);

    // Verify WHO_AM_I
    ICM20948_ReadRegister(&hi2c1, ICM20948_WHO_AM_I, &data);
    if (data != 0xEA) {
        return HAL_ERROR; // Failed to detect ICM20948
    }
    _ICM20948_Gyro_Calib(imu_data);
    printf("ICM20948 done gyro calib\r\n");
    return Magnetometer_Init();
}

void ICM20948_ReadDMA(ICM20948_t* imu_data) {
    // Trigger DMA read for accel and gyro data (14 bytes total)
    HAL_I2C_Mem_Read_DMA(&hi2c1, ICM20948_ADDR, ICM20948_ACCEL_XOUT_H, 1, imu_data->acc_gyro_buff, 14);
}
void ICM20948_Process_Gyro_data(ICM20948_t* imu_data)
{
	uint8_t *buff = imu_data->acc_gyro_buff;
	// Read Accelerometer Data
	imu_data->acce_raw[0] = (int16_t)((buff[0] << 8) | buff[1]);
	imu_data->acce_raw[1] = (int16_t)((buff[2] << 8) | buff[3]);
	imu_data->acce_raw[2] = (int16_t)((buff[4] << 8) | buff[5]);
	for(uint8_t i = 0; i < 3; ++i)
		imu_data->acce[i] = imu_data->acce_raw[i] / 16384.0f; // Convert raw data to g
	// Read Gyroscope Data
	imu_data->gyro_raw[0] = (int16_t)((buff[8] << 8) | buff[9]);
	imu_data->gyro_raw[1] = (int16_t)((buff[10] << 8) | buff[11]);
	imu_data->gyro_raw[2] = (int16_t)((buff[12] << 8) | buff[13]);
	for(uint8_t i = 0; i < 3; ++i)
	{
		imu_data->gyro[i] = (imu_data->gyro_raw[i] - imu_data->gyro_offset[i]) / 65.5f; // Convert raw data to dps
	}

}

uint8_t Magnetometer_Init(void)
{
	uint8_t mag_data;

	// Switch to Bank 0
	ICM20948_SwitchBank(&hi2c1, ICM20948_USER_BANK_0);

	// Enable I2C bypass mode to directly access the magnetometer
	ICM20948_WriteRegister(&hi2c1, REG_ADD_INT_PIN_CFG, REG_VAL_BIT_I2C_BYPASS_EN);

	// Wait briefly to stabilize
	HAL_Delay(10);

	// Check magnetometer WHO_AM_I register directly
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, ICM20948_MAG_ADDR, AK09916_WIA2, 1, &mag_data, 1, HAL_MAX_DELAY);
	if (status != HAL_OK || mag_data != 0x09) {
		return HAL_ERROR; // Magnetometer not detected
	}

	// Configure the magnetometer to continuous measurement mode
	uint8_t mag_ctrl_data = AK09916_CNTL2_MODE_CONT;
	status = HAL_I2C_Mem_Write(&hi2c1, ICM20948_MAG_ADDR, AK09916_CNTL2, 1, &mag_ctrl_data, 1, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return HAL_ERROR; // Failed to configure magnetometer
	}

	// Wait for the magnetometer to stabilize
	HAL_Delay(10);

	return HAL_OK; // Magnetometer initialized successfully
}
uint8_t Magnetometer_Master_Init(void) {
    uint8_t mag_data = 0;

    // Switch to Bank 0
    if (ICM20948_SwitchBank(&hi2c1, ICM20948_USER_BANK_0) != HAL_OK) {
        printf("Failed to switch to Bank 0\n");
        return HAL_ERROR;
    }

    // Enable I2C master mode in USER_CTRL
    if (ICM20948_WriteRegister(&hi2c1, REG_ADD_USER_CTRL, REG_VAL_BIT_I2C_MST_EN) != HAL_OK) {
        printf("Failed to enable I2C master mode\n");
        return HAL_ERROR;
    }

    // Switch to Bank 3
    if (ICM20948_SwitchBank(&hi2c1, ICM20948_USER_BANK_3) != HAL_OK) {
        printf("Failed to switch to Bank 3\n");
        return HAL_ERROR;
    }

    // Configure Slave 0 to read WHO_AM_I
    if (ICM20948_WriteRegister(&hi2c1, REG_ADD_I2C_SLV0_ADDR, ICM20948_MAG_ADDR | 0x80) != HAL_OK) {
        printf("Failed to configure Slave 0 address\n");
        return HAL_ERROR;
    }
    if (ICM20948_WriteRegister(&hi2c1, REG_ADD_I2C_SLV0_REG, AK09916_WIA2) != HAL_OK) {
        printf("Failed to configure Slave 0 register\n");
        return HAL_ERROR;
    }
    if (ICM20948_WriteRegister(&hi2c1, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | 1) != HAL_OK) {
        printf("Failed to configure Slave 0 control\n");
        return HAL_ERROR;
    }

    // Switch back to Bank 0
    if (ICM20948_SwitchBank(&hi2c1, ICM20948_USER_BANK_0) != HAL_OK) {
        printf("Failed to switch to Bank 0\n");
        return HAL_ERROR;
    }

    // Delay to allow data to populate
    HAL_Delay(10);

    // Read the external sensor data
    if (ICM20948_ReadRegister(&hi2c1, REG_ADD_EXT_SENS_DATA_00, &mag_data) != HAL_OK) {
        printf("Failed to read external sensor data\n");
        return HAL_ERROR;
    }

    // Verify WHO_AM_I value
    printf("Magnetometer WHO_AM_I: 0x%02X\n", mag_data);
    if (mag_data != 0x09) {
        printf("Magnetometer not detected\n");
        return HAL_ERROR;
    }

    // Switch to Bank 3 for further configuration
    if (ICM20948_SwitchBank(&hi2c1, ICM20948_USER_BANK_3) != HAL_OK) {
        printf("Failed to switch to Bank 3\n");
        return HAL_ERROR;
    }

    // Set magnetometer to continuous mode
    if (ICM20948_WriteRegister(&hi2c1, REG_ADD_I2C_SLV0_ADDR, ICM20948_MAG_ADDR) != HAL_OK) {
        printf("Failed to configure Slave 0 address for write\n");
        return HAL_ERROR;
    }
    if (ICM20948_WriteRegister(&hi2c1, REG_ADD_I2C_SLV0_REG, AK09916_CNTL2) != HAL_OK) {
        printf("Failed to configure Slave 0 register for write\n");
        return HAL_ERROR;
    }
    if (ICM20948_WriteRegister(&hi2c1, REG_ADD_I2C_SLV0_DO, AK09916_CNTL2_MODE_CONT) != HAL_OK) {
        printf("Failed to configure Slave 0 data for write\n");
        return HAL_ERROR;
    }
    if (ICM20948_WriteRegister(&hi2c1, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | 1) != HAL_OK) {
        printf("Failed to configure Slave 0 control for write\n");
        return HAL_ERROR;
    }

    // Switch back to Bank 0
    if (ICM20948_SwitchBank(&hi2c1, ICM20948_USER_BANK_0) != HAL_OK) {
        printf("Failed to switch to Bank 0\n");
        return HAL_ERROR;
    }

    // Delay to allow the magnetometer to stabilize
    HAL_Delay(10);

    return HAL_OK; // Success
}


void Magnetometer_ReadDMA(ICM20948_t* imu_data) {
    // Trigger DMA read for magnetic data
    HAL_I2C_Mem_Read_DMA(&hi2c1, ICM20948_MAG_ADDR, AK09916_HXL, 1, imu_data->magn_buff, 6);
}

void ICM20948_Process_Mage_data(ICM20948_t* imu_data)
{
	uint8_t *buff = imu_data->magn_buff;
	float *mag = imu_data->mage;
	// Read Magnetometer Data
	mag[0] = (int16_t)((buff[1] << 8) | buff[0]) * 0.15f; // Convert raw data to µT
	mag[1] = (int16_t)((buff[3] << 8) | buff[2]) * 0.15f;
	mag[2] = (int16_t)((buff[5] << 8) | buff[4]) * 0.15f;
}
