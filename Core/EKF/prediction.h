/*
 * prediction.h
 *
 *  Created on: May 16, 2026
 *      Author: alexk
 */

#ifndef EKF_PREDICTION_H_
#define EKF_PREDICTION_H_

#include "arm_math.h"
#include "ekf.h"
#include "..\sensors\icm40609d.h"


void predictAttitude(ekfState *inst,float32_t dt,ICM40609D_Data_t *icmData);

#endif /* EKF_PREDICTION_H_ */
