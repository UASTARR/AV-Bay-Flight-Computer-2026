/*
 * ekf.h
 *
 *  Created on: May 16, 2026
 *      Author: alexk
 */

#ifndef EKF_EKF_H_
#define EKF_EKF_H_

#include "arm_math.h"

typedef struct {
	float32_t q[4]; //Attitude
	float32_t qold[4]; //Attitude old

	float32_t P[16]; //Process covariance matrix
	float32_t Pold[16]; //Old process covariance matrix

	float32_t K[16];

	float32_t rnsd_w;
} ekfState;

#endif /* EKF_EKF_H_ */
