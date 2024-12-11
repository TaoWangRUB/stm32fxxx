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
    if(norm_factor < 1e-10) return;

    for (uint8_t i = 0; i < 3; ++i) {
        acce[i] /= norm_factor;
    }

    // Calculate pitch and roll
    angle[0] = atan2f(acce[1], sqrt(acce[0] * acce[0] + acce[2] * acce[2])) * RAD_TO_DEG; // Pitch
    angle[1] = atan2f(acce[0], sqrt(acce[1] * acce[1] + acce[2] * acce[2])) * RAD_TO_DEG; // Roll
    angle[2] = 0; // Yaw requires magnetometer or gyro data

    return;
}

void angle_estimation_kalman(float gyro, float angle, float *kalmanOutput)
{
	float state = kalmanOutput[0];
	float uncert = kalmanOutput[1];

	const float dt = 10 * 0.001; // from microsec to sec
	const float error_rate = 4;		// process error is 4 deg/s
	const float error_angle = 3;	// measurement error 3 deg

	// step 1. predict new state
	state += gyro * dt;
	// step 2. calculate uncertainty of prediction
	uncert += dt * dt * error_rate * error_rate;
	// step 3. calculate kalman gain
	float gain = uncert / (uncert + error_angle * error_angle);
	// step 4. correction with kalman gain
	state += gain * (angle - state);
	// step 5. calculate uncertainty
	uncert *= (1 - gain);

	kalmanOutput[0] = state;
	kalmanOutput[1] = uncert;
	kalmanOutput[2] = gain;
}
