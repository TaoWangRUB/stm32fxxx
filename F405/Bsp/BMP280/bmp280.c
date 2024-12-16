/*
 * bmp280.c
 *
 *  Created on: Dec 10, 2024
 *      Author: taowang
 */


#include "bmp280.h"
#include "i2c.h"

// Helper functions
static HAL_StatusTypeDef BMP280_ReadRegister(uint8_t reg, uint8_t *data, uint16_t length) {
    return HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef BMP280_WriteRegister(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

static calibrate_altitude(BMP280_t *bmp) {
	const uint16_t Num = 1000;
	for (uint16_t i = 0; i < Num; ++i) {
		BMP280_ReadTempAndPressure(bmp);
		//BMP280_ReadPressure(bmp, &(bmp->press));
		bmp->altitude_offset += bmp->altitude;
		HAL_Delay(10);
	}
	bmp->altitude_offset /= Num;
	printf("Altitude offset: %.5f\r\n", bmp->altitude_offset);
	return;
}
// BMP280 Initialization
uint8_t BMP280_Init(BMP280_t *bmp) {
    uint8_t id;
    uint8_t config[2];

    // Read the BMP280 ID
    if (BMP280_ReadRegister(BMP280_REG_ID, &id, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (id != 0x58) {
        return HAL_ERROR; // BMP280 ID mismatch
    }

    // Reset the sensor
    if (BMP280_WriteRegister(BMP280_REG_RESET, 0xB6) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(100);

    // Set oversampling and power mode
    config[0] = BMP280_OSRS_T | BMP280_OSRS_P | BMP280_MODE_NORMAL;
    config[1] = 0x14; // Standby time 1000ms, filter on
    if (BMP280_WriteRegister(BMP280_REG_CTRL_MEAS, config[0]) != HAL_OK ||
        BMP280_WriteRegister(BMP280_REG_CONFIG, config[1]) != HAL_OK) {
        return HAL_ERROR;
    }

    // Read calibration data
    uint8_t calib[BMP280_CALIB_LENGTH];
    if (BMP280_ReadRegister(BMP280_CALIB_START, calib, BMP280_CALIB_LENGTH) != HAL_OK) {
        return HAL_ERROR;
    }

    // Parse calibration data
    bmp->calib.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    bmp->calib.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    bmp->calib.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);

    bmp->calib.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    bmp->calib.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    bmp->calib.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    bmp->calib.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    bmp->calib.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    bmp->calib.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    bmp->calib.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    bmp->calib.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    bmp->calib.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
    HAL_Delay(100);

    // Calibrate altitude
    calibrate_altitude(bmp);
    printf("BMP280 done altitude calib\r\n");

    return HAL_OK;
}

// BMP280 Temperature Compensation
static int32_t BMP280_CompensateTemperature(BMP280_t *bmp, int32_t adc_T) {
    int32_t var1, var2;
    var1 = (((adc_T >> 3) - ((int32_t)bmp->calib.dig_T1 << 1)) * ((int32_t)bmp->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp->calib.dig_T1))) >> 12) * ((int32_t)bmp->calib.dig_T3)) >> 14;
    bmp->t_fine = var1 + var2;
    return (bmp->t_fine * 5 + 128) >> 8;
}

// BMP280 Pressure Compensation
static uint32_t BMP280_CompensatePressure(BMP280_t *bmp, int32_t adc_P) {
    int64_t var1, var2, p;
#if 1
    var1 = ((int64_t)bmp->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp->calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp->calib.dig_P1) >> 33;
    if (var1 == 0) return 0; // avoid division by zero
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->calib.dig_P7) << 4);
    return (uint32_t)p;
#else
    var1 = (((int64_t)bmp->t_fine)>>1) - (int64_t)64000;
      var2 = (((var1>>2) * (var1>>2)) >> 11 ) *((int64_t)bmp->calib.dig_P6);
      var2 = var2 + ((var1 *((int64_t)bmp->calib.dig_P5))<<1);
      var2 = (var2>>2) + (((int64_t)bmp->calib.dig_P4)<<16);
      var1 = (((bmp->calib.dig_P3 * (((var1>>2) * (var1>>2))>>13))>>3) + ((((int64_t)bmp->calib.dig_P2) * var1)>>1))>>18;
      var1 = ((((32768+var1))*((int64_t)bmp->calib.dig_P1))>>15);
      if(var1 ==0)
      {
        return 0;
      }
      p = (1048576.0 - adc_P) - (var2>>12)*3125;
      if(p < 0x80000000)
      {
        p = (p<<1)/((uint64_t)var1);
      }
      else
      {
        p = (p/(uint64_t)var1)*2;
      }
      var1 = (((int64_t)bmp->calib.dig_P9) *((int64_t)(((p>>3)*(p>>3))>>13)))>>12;
      var2 = (((int64_t)(p>>2))*((int64_t)bmp->calib.dig_P8))>>13;
      p = (uint64_t)((int64_t)p) +((var1 + var2 + bmp->calib.dig_P7)>>4);
      return p;
#endif
}

//
uint8_t BMP280_ReadTempAndPressure(BMP280_t *bmp)
{
	if (BMP280_ReadRegister(BMP280_REG_PRESS_MSB, bmp->buff, 6) != HAL_OK) {
		return HAL_ERROR;
	}
	BMP280_Process_data(bmp);
	return HAL_OK;
}
// Read Temperature and Pressure
void BMP280_Process_data(BMP280_t *bmp) {
	uint8_t *data = bmp->buff;

	int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);
	int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

	// Compensate temperature
	bmp->temp = BMP280_CompensateTemperature(bmp, adc_T) / 100.f;
	// Compensate pressure
	bmp->press = BMP280_CompensatePressure(bmp, adc_P) / 100.0f;
	// calculate altitude
	bmp->altitude = 44330.0f * (1.0f - pow((bmp->press / 1013.25f), 0.1903f));
	return;
}
// Read Temperature
uint8_t BMP280_ReadTemperature(BMP280_t *bmp, float *temperature) {
	uint8_t data[3];
	if (BMP280_ReadRegister(BMP280_REG_TEMP_MSB, data, 3) != HAL_OK) {
		return HAL_ERROR;
	}

	int32_t adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

	// Compensate temperature
	int32_t temperature_raw = BMP280_CompensateTemperature(bmp, adc_T);
	*temperature = (float)temperature_raw / 100.0f;

	return HAL_OK;
}

// Read Pressure
uint8_t BMP280_ReadPressure(BMP280_t *bmp, float *pressure) {
	uint8_t data[3];
	if (BMP280_ReadRegister(BMP280_REG_PRESS_MSB, data, 3) != HAL_OK) {
		return HAL_ERROR;
	}

	int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

	// Compensate pressure
	*pressure = (float)BMP280_CompensatePressure(bmp, adc_P) / 100.0f;

	return HAL_OK;
}

void BMP280_Read_DMA(BMP280_t *bmp)
{
	HAL_I2C_Mem_Read_DMA(&hi2c1, BMP280_I2C_ADDR, BMP280_REG_PRESS_MSB, I2C_MEMADD_SIZE_8BIT, bmp->buff, 6);
}
