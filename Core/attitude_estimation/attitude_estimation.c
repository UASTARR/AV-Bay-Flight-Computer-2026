/*
 * attitude_estimation.c
 *
 *  Created on: May 15, 2026
 *      Author: alexk
 */

#include "arm_math.h"
#include "attitude_estimation.h"
#include "..\sensors\icm40609d.h"
#include "..\sensors\mmc5983ma.h"
#include "..\EKF\ekf.h"

void qernionAttitudeEst(quat *q,ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData) {

	float imag = 1/sqrtf(powf(icmData->accel_x,2) + powf(icmData->accel_y,2) + powf(icmData->accel_z,2));
	float mmag = 1/sqrtf(powf(magData->x,2) + powf(magData->y,2) + powf(magData->z,2));

	icmData->accel_x = icmData->accel_x*imag;
	icmData->accel_y = icmData->accel_y*imag;
	icmData->accel_z = icmData->accel_z*imag;

	magData->x = magData->x*mmag;
	magData->y = magData->y*mmag;
	magData->z = magData->z*mmag;

	float mD = (icmData->accel_x*magData->x) + (icmData->accel_y*magData->y) + (icmData->accel_z*magData->z);
	float mN = sqrtf(1-powf(mD,2));

	float qw = -icmData->accel_y*(mN+magData->x) + icmData->accel_x*magData->y;
	float qx = (icmData->accel_z-1)*(mN+magData->x) + (icmData->accel_x)*(mD-magData->z);
	float qy = (icmData->accel_z-1)*magData->y + (icmData->accel_y)*(mD-magData->z);
	float qz = (icmData->accel_z * mD) - (icmData->accel_x * mN) - magData->z;

	float qmag = 1/sqrtf(powf(qw,2) + powf(qx,2) + powf(qy,2) + powf(qz,2));
	q->qw = qw*qmag;
	q->qx = qx*qmag;
	q->qy = qy*qmag;
	q->qz = qz*qmag;
}

void AttitudeEst(eulerAngle *ang,ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData) {
	quat q = {0};
	qernionAttitudeEst(&q,icmData,magData);

	ang->x = atan2(2*(q.qw*q.qx+q.qy*q.qz), 1-2*(powf(q.qx,2) + powf(q.qy,2)));
	ang->y = asin(2*(q.qw*q.qy - q.qz*q.qx));
	ang->z = atan2(2*(q.qw*q.qz+q.qx*q.qy), 1-2*(powf(q.qy,2) + powf(q.qz,2)));
}
