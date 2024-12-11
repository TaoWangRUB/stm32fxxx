#ifndef MPU6050_H
#define MPU6050_H

#define MPU6050_USE_DMA 1   // 0: not used I2C DMA mode, 1: used I2C DMA mode

// MPU6050 structure
typedef struct
{
	uint8_t mpu6050_raw[14];
    int16_t acce_raw[3];
    float acce[3];

    int16_t gyro_raw[3];
    float gyro[3];
    float gyro_offset[3];

    float Temperature;

    float angle[3];
    uint8_t is_reading;
} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Read_DMA(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Process_Data(MPU6050_t *DataStruct);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif /* MPU6050_H */
