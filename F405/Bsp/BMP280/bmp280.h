/*
 * bmp280.h
 *
 *  Created on: Dec 10, 2024
 *      Author: taowang
 */

#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>

// BMP280 I2C Address
#define BMP280_I2C_ADDR 0x77 << 1 // Default address (can also be 0x77 << 1 depending on SDO pin)

// BMP280 Registers
#define BMP280_REG_ID              0xD0
#define BMP280_REG_RESET           0xE0
#define BMP280_REG_CTRL_MEAS       0xF4
#define BMP280_REG_CONFIG          0xF5
#define BMP280_REG_PRESS_MSB       0xF7
#define BMP280_REG_TEMP_MSB        0xFA

// Calibration parameter registers (start address)
#define BMP280_CALIB_START         0x88
#define BMP280_CALIB_LENGTH        24

// Oversampling and power modes
#define BMP280_OSRS_T              0x20 // Temperature oversampling x1
#define BMP280_OSRS_P              0x04 // Pressure oversampling x1
#define BMP280_MODE_NORMAL         0x03 // Normal mode

// Structure to store calibration data
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} BMP280_CalibData;

// Structure for BMP280 driver
typedef struct {
    BMP280_CalibData calib;  // Calibration data
    int32_t t_fine;          // Fine temperature for pressure calculation
    uint8_t buff[6];
    float temp;
    float press;
} BMP280_t;

// Function prototypes
uint8_t BMP280_Init(BMP280_t *bmp);
uint8_t BMP280_ReadTemperature(BMP280_t *bmp, float *temperature);
uint8_t BMP280_ReadPressure(BMP280_t *bmp, float *pressure);
uint8_t BMP280_ReadTempAndPressure(BMP280_t *bmp);
void BMP280_Process_data(BMP280_t *bmp);
void BMP280_Read_DMA(BMP280_t *bmp);

#endif // BMP280_H

