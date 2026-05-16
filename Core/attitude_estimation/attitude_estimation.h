/*
 * attitude_estimation.h
 *
 *  Created on: May 15, 2026
 *      Author: alexk
 */

#ifndef ATTITUDE_ESTIMATION_ATTITUDE_ESTIMATION_H_
#define ATTITUDE_ESTIMATION_ATTITUDE_ESTIMATION_H_

#include "..\sensors\icm40609d.h"
#include "..\sensors\mmc5983ma.h"

typedef struct {
	float qw;
	float qx;
	float qy;
	float qz;
} quaternion;

typedef struct {
	float x;
	float y;
	float z;
} eulerAngle;

void quaternionAttitudeEst(quaternion *quat,ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData);
void AttitudeEst(eulerAngle *ang,ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData);

#endif /* ATTITUDE_ESTIMATION_ATTITUDE_ESTIMATION_H_ */
