/*
 * angle_estimation.c
 *
 *  Created on: Dec 11, 2024
 *      Author: taowang
 */

#include "algo/angle_estimation.h"
#include <math.h>
#include <stdint.h>

#define RAD_TO_DEG 57.29577951308232 // 180 / pi

void angle_estimation(float *acce, float *angle)
{
    // Normalize accelerometer values
    float norm_factor = sqrt(acce[0] * acce[0] + acce[1] * acce[1] + acce[2] * acce[2]);
    for (uint8_t i = 0; i < 3; ++i) {
        acce[i] /= norm_factor;
    }

    // Calculate pitch and roll
    angle[0] = atan2f(acce[1], sqrt(acce[0] * acce[0] + acce[2] * acce[2])) * RAD_TO_DEG; // Pitch
    angle[1] = atan2f(acce[0], sqrt(acce[1] * acce[1] + acce[2] * acce[2])) * RAD_TO_DEG; // Roll
    angle[2] = 0; // Yaw requires magnetometer or gyro data

    return;
}

