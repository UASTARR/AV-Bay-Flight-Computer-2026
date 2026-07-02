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

// Estimated offsets based on your minimums and maximums
// Initialize max to lowest possible float, and min to highest possible float
static float mag_min[3] = { 99999.0f,  99999.0f,  99999.0f};
static float mag_max[3] = {-99999.0f, -99999.0f, -99999.0f};
static float mag_bias[3] = {0.0f, 0.0f, 0.0f};

float32_t clip(float32_t val, float min, float max) {
	return (float32_t) (val<min?min:(val>max?max:val));
}

int sign(float32_t val) {
	return (val>=0)?1:-1;
}

void quatInverse(float32_t *qIn,float32_t *qOut) {
	qOut[0] = qIn[0];
	qOut[1] = -qIn[1];
	qOut[2] = -qIn[2];
	qOut[3] = -qIn[3];
}

void quatProduct(const float32_t *q, const float32_t *r, float32_t *t_out) {
    float32_t t[4];

    t[0] = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3]; // w component
    t[1] = q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2]; // x component
    t[2] = q[0]*r[2] - q[1]*r[3] + q[2]*r[0] + q[3]*r[1]; // y component
    t[3] = q[0]*r[3] + q[1]*r[2] - q[2]*r[1] + q[3]*r[0]; // z component

    memcpy(t_out, t, 4 * sizeof(float32_t));
}

void qernionAttitudeEst(float32_t *q,ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData) {
	//Normalize
	//	amag = math.sqrt(a[0]**2+a[1]**2+a[2]**2)
	//	a = [x/amag for x in a] if amag!=0 else [0,0,0]
	float accelMag = sqrtf(powf(icmData->accel_x,2.0) + powf(icmData->accel_y,2.0) + powf(icmData->accel_z,2.0));
	float32_t a[3] = {-icmData->accel_x/accelMag, -icmData->accel_y/accelMag,-icmData->accel_z/accelMag};
	//float32_t a[3] = {0.0,0.0,1.0};

	// 1. Log Raw Data
	float raw_m[3] = {magData->x, magData->y, magData->z};

	// 2. Track Boundaries dynamically
	for(int i = 0; i < 3; i++) {
		if (!(raw_m[i]>100 || raw_m[i]<-100)) {
			if(raw_m[i] < mag_min[i]) mag_min[i] = raw_m[i];
			if(raw_m[i] > mag_max[i]) mag_max[i] = raw_m[i];
		}

	    // Calculate the midpoint offset (Hard-Iron Bias)
	    mag_bias[i] = (mag_max[i] + mag_min[i]) / 2.0f;
	}

	// 3. Apply the Hard-Iron correction BEFORE normalization matrix calculations
	float32_t cmag_x = magData->x - mag_bias[0];
	float32_t cmag_y = magData->y - mag_bias[1];
	float32_t cmag_z = magData->z - mag_bias[2];
	//	mmag = math.sqrt(m[0]**2+m[1]**2+m[2]**2)
	//	m = [x/mmag for x in m] if mmag!=0 else [0,0,0]
	float magMag = sqrtf(cmag_x*cmag_x + cmag_y*cmag_y + cmag_z*cmag_z);
	float32_t m[3] = {cmag_x/magMag, cmag_y/magMag, cmag_z/magMag};
	//float32_t m[3] = {1.0,1.0,1.0};

	//Elevation
	//	q = [0,0,0,0]
	//	stheta = np.clip(a[0], -1.0, 1.0)
	//	ctheta = np.sqrt(1.0-stheta**2)
	//	qe = np.array([np.sqrt((1.0+ctheta)/2.0) , 0.0, np.sign(stheta)*np.sqrt((1.0-ctheta)/2.0), 0.0])
	float32_t stheta = clip(a[0],-1.0,1.0);
	float32_t ctheta = sqrtf(1.0-stheta*stheta);
	float32_t qe[4] = {sqrtf((1.0+ctheta)/2.0),0.0,sign(stheta)*sqrtf((1.0-ctheta)/2.0),0.0};

	//Roll
	//	sphi = np.clip(-a[1]/ctheta,-1,1) if ctheta!=0 else 0
	//	cphi = np.clip(-a[2]/ctheta,-1,1) if ctheta!=0 else 0
	//	rsign = 1 if sphi>=0 else -1
	//	if cphi==-1.0 and sphi==0.0:
	//		rsign = 1.0
	//	qr = np.array([math.sqrt((1+cphi)/2),rsign*math.sqrt((1-cphi)/2),0,0])
	float32_t sphi = 0.0f;
	float32_t cphi = 1.0f; // Default identity state (0 degrees roll)

	// Check if we are safely clear of the 90-degree vertical singularity
	if (ctheta > 0.001f) {
		sphi = clip(-a[1] / ctheta, -1.0f, 1.0f);
		cphi = clip(-a[2] / ctheta, -1.0f, 1.0f);
	} else {
		// Singularity handling: The board is vertical!
		// Force roll to 0 to prevent division by zero; yaw will handle the rotation.
		sphi = 0.0f;
		cphi = 1.0f;
	}
	int rsign = sign(sphi);
	if ((cphi == -1.0) && (sphi == 0.0)) {
		rsign = 1;
	}
	float32_t qr[4] = {sqrtf((1.0+cphi)/2.0),rsign*sqrtf((1.0-cphi)/2.0),0.0,0.0};

	//Azimuth
	//	mb = np.array([0,m[0],m[1],m[2]])
	//	me = quatProduct(quatInverse(qr),quatInverse(qe))
	//	me = quatProduct(mb,me)
	//	me = quatProduct(qr,me)
	//	me = quatProduct(qe,me)
	//	M = np.array([me[1],me[2]])/np.sqrt(me[1]**2 + me[2]**2)
	//	N = np.array([0.9983779952066275, 0.05693310712753428])
	//	#N = np.array([14.408,3.440])/56.141
	//	cpsi, spsi = np.array([[M[0], M[1]], [-M[1], M[0]]])@N
	//	asign = 1 if spsi>=0 else -1
	//	qa = np.array([math.sqrt((1+cpsi)/2),0,0,asign*math.sqrt((1-cpsi)/2)])
	//	q = quatProduct(qe,qr)
	//	q = quatProduct(qa,q)
	//	return q
	float32_t mb[4] = {0.0,m[0],m[1],m[2]};
	float32_t me[4];
	// Step A: Calculate the total tilt quaternion: q_tilt = q_e * q_r
	float32_t q_tilt[4];
	quatProduct(qe, qr, q_tilt);

	// Step B: Calculate the inverse tilt quaternion
	float32_t q_tilt_inv[4];
	quatInverse(q_tilt, q_tilt_inv);

	// Step C: Perform the vector rotation sandwich: me = q_tilt_inv * mb * q_tilt
	float32_t temp[4];
	quatProduct(q_tilt_inv, mb, temp);   // Multiply left side
	quatProduct(temp, q_tilt, me);       // Multiply right side to get the final me vector

	float32_t meMag = sqrtf(powf(me[1],2.0) + powf(me[2],2.0));
	// Add protection for the magnetometer divisor step
	if (meMag < 0.000001f) {
		meMag = 0.000001f;
	}
	float32_t M[2] = {me[1]/meMag,me[2]/meMag};
	float32_t N[2] = {0.9983779952066275, 0.05693310712753428}; //Magic Numbers for now, make an easy way to change these
	float32_t cpsi = clip(M[0]*N[0] + M[1]*N[1],-1.0,1.0);
	float32_t spsi = clip(-M[1]*N[0] + M[0]*N[1],-1.0,1.0);
	int asign = sign(spsi);
	float32_t qa[4] = {sqrtf((1.0+cpsi)/2.0),0.0,0.0,asign*sqrtf((1.0-cpsi)/2.0)};

	quatProduct(qe,qr,q);
	quatProduct(qa,q,q);
	float qmag = sqrtf(powf(q[0],2) + powf(q[1],2) + powf(q[2],2) + powf(q[3],2));
	q[0] = q[0]/qmag;
	q[1] = q[1]/qmag;
	q[2] = q[2]/qmag;
	q[3] = q[3]/qmag;
}
