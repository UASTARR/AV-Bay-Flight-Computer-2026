/*
 * ekf.h
 *
 *  Created on: May 16, 2026
 *      Author: alexk
 */

#ifndef EKF_EKF_H_
#define EKF_EKF_H_

#include "prediction.h"

typedef struct {
	quat q; //Attitude
	quat qold; //Attitude old
} ekfState;

typedef struct {
	float32_t qw;
	float32_t qx;
	float32_t qy;
	float32_t qz;
} quat;


#endif /* EKF_EKF_H_ */
