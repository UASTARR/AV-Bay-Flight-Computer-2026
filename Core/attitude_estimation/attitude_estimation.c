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

void quaternionAttitudeEst(quaternion *quat,ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData) {

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
	quat->qw = qw*qmag;
	quat->qx = qx*qmag;
	quat->qy = qy*qmag;
	quat->qz = qz*qmag;
}

void AttitudeEst(eulerAngle *ang,ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData) {
	quaternion quat = {0};
	quaternionAttitudeEst(&quat,icmData,magData);

	ang->x = atan2(2*(quat.qw*quat.qx+quat.qy*quat.qz), 1-2*(powf(quat.qx,2) + powf(quat.qy,2)));
	ang->y = asin(2*(quat.qw*quat.qy - quat.qz*quat.qx));
	ang->z = atan2(2*(quat.qw*quat.qz+quat.qx*quat.qy), 1-2*(powf(quat.qy,2) + powf(quat.qz,2)));
}
