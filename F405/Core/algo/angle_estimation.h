/*
 * angle_estimation.h
 *
 *  Created on: Dec 11, 2024
 *      Author: taowang
 */

#ifndef ALGO_ANGLE_ESTIMATION_H_
#define ALGO_ANGLE_ESTIMATION_H_

void angle_estimation(float *acce, float *angle);
void angle_estimation_kalman(float gyro, float angle, float *kalmanOutput);
#endif /* ALGO_ANGLE_ESTIMATION_H_ */
