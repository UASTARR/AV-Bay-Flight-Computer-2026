/*
 * filter.c
 *
 *  Created on: May 16, 2026
 *      Author: alexk
 */


#include <stdio.h>
#include "arm_math.h"
#include "prediction.h"
#include "ekf.h"
#include "..\sensors\icm40609d.h"
#include "..\sensors\mmc5983ma.h"


void crossProd(float32_t *a, float32_t *b, float32_t *c) {
	float32_t cnew[3];
	cnew[0] = a[1]*b[2] - a[2]*b[1];
	cnew[1] = a[2]*b[0] - a[0]*b[2];
	cnew[2] = a[0]*b[1] - a[1]*b[0];
	memcpy(c,cnew,3*sizeof(float32_t));
}

void outerProd(float32_t *a, float32_t *b, float32_t *c) {
	float32_t cnew[9];
	arm_matrix_instance_f32 A;
	arm_mat_init_f32(&A,3,1,a);

	arm_matrix_instance_f32 B;
	arm_mat_init_f32(&B,1,3,b);

	arm_matrix_instance_f32 C;
	arm_mat_init_f32(&C,3,3,c);
	arm_mat_mult_f32(&A,&B,&C);
}

void init(ekfState *inst) {
	//Set P_0 as identity matrix
	for(int i=0; i<4; i++) {
		for(int j=0; j<4; j++) {
			if(i==j) {
				inst->P[i+j] = 1;
			} else {
				inst->P[i+j] = 0;
			}
		}
	}

	//Set intial estimate of q
	ICM40609D_Data_t icm = {0};
	MMC5983MA_Data_t mag = {0};

	ICM40609D_Read_All(&icm);
	MMC5983MA_Read_All(&mag);

	fastqEstimation(inst,&icm,&mag);

	//Set Rate Noise Spectral Densities
	inst->rnsd_w = powf(0.0000785,2); //(rad^2)/(s^2 * hz)
}

void predictAttitude(ekfState *inst,float32_t dt,ICM40609D_Data_t *icmData) {
	//Update t-1 variables
	memcpy(inst->qold,inst->q,4*sizeof(float32_t));
	memcpy(inst->Pold,inst->P,16*sizeof(float32_t));


	//Init vector of gyro data
	float32_t w[3] = {icmData->gyro_x,icmData->gyro_y,icmData->gyro_z};

	//Init identity matrix
	float32_t Idata[16] = {
			1,0,0,0,
			0,1,0,0,
			0,0,1,0
	};
	arm_matrix_instance_f32 I;
	arm_mat_init_f32(&I,4,4,Idata);

	//Init Omega_t
	float32_t omegaData[16] = {
			0,   -w[0],-w[1],-w[2],
			w[0],  0,   w[2],-w[1],
			w[1],-w[2], 0,    w[0],
			w[2], w[1],-w[0], 0
	};
	arm_matrix_instance_f32 omega;
	arm_mat_init_f32(&omega,4,4,omegaData);
	arm_mat_scale_f32(&omega,dt/2,&omega); //Scale by dt/2

	//Init jacobian matrix
	arm_matrix_instance_f32 F;
	float32_t FData[16];
	arm_mat_init_f32(&F,4,4,FData);

	//Calculate 2nd order Taylor polynomial
	arm_mat_add_f32(&I,&omega,&F); //Add first term
	arm_mat_mult_f32(&omega,&omega,&omega); //Square omega
	arm_mat_scale_f32(&omega,0.5,&omega); //Multiply omega by 1/2! for second term
	arm_mat_add_f32(&F,&omega,&F); //Add new omega to F

	//Update state
	arm_matrix_instance_f32 q;
	arm_mat_init_f32(&q,4,1,inst->q);
	arm_matrix_instance_f32 qold;
	arm_mat_init_f32(&qold,4,1,inst->qold);

	arm_mat_mult_f32(&F,&qold,&q); //Multiply the F by the previous state


	//Init Q_t
	arm_matrix_instance_f32 Q;
	float32_t QData[16];
	arm_mat_init_f32(&Q,4,4,QData);

	//Init W
	arm_matrix_instance_f32 W;
	float32_t WData[12] ={
			-inst->q[1],-inst->q[2],-inst->q[3],
			 inst->q[0],-inst->q[3], inst->q[2],
			 inst->q[3], inst->q[0],-inst->q[1],
			-inst->q[2], inst->q[1], inst->q[0]
	};
	arm_mat_init_f32(&W,4,3,WData);
	arm_mat_scale_f32(&W,dt/2,&W); //Scale by dt/2

	arm_matrix_instance_f32 Sigma;
		//Add more here once we figure out wtf this is
	float32_t SigmaData[9] = {
			inst->rnsd_w*dt, 0, 0,
			0, inst->rnsd_w*dt, 0,
			0, 0, inst->rnsd_w*dt
	};
	arm_mat_init_f32(&Sigma,3,3,SigmaData);

	//Compute Q = W * sigma * W^T
	arm_mat_mult_f32(&W,&Sigma,&Q);
	arm_mat_trans_f32(&W,&W);
	arm_mat_mult_f32(&Q,&W,&Q);

	//Update process noise covariance
	arm_matrix_instance_f32 Pold;
	arm_mat_init_f32(&Pold,4,4,inst->Pold);

	arm_matrix_instance_f32 P;
	arm_mat_init_f32(&P,4,4,inst->P);

	//Calculate F * P_(t-1) * F^T
	arm_mat_mult_f32(&F,&Pold,&Pold);
	arm_mat_trans_f32(&F,&F);
	arm_mat_mult_f32(&Pold,&F,&Pold);

	//Add result of previous step to Q to get P_t
	arm_mat_add_f32(&Pold,&Q,&P);
}

void correctAttitude(ekfState *inst, ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData) {
	//Measurement Vector 1x6
	float32_t z[6] = {icmData->accel_x,icmData->accel_y,icmData->accel_z,magData->x,magData->y,magData->z};

	//State quaternion
	arm_matrix_instance_f32 Q;
	float32_t q[4];
	memcpy(q,inst->q,4*sizeof(float32_t));
	arm_mat_init_f32(&Q,4,1,inst->q);

	//Process Covariance Matrix
	arm_matrix_instance_f32 P;
	arm_mat_init_f32(&P,4,4,inst->P);

	//Kalman Gain Matrix
	arm_matrix_instance_f32 K;
	arm_mat_init_f32(&K,4,6,inst->K);

	//NED gravitation field
	float32_t g[3] = {0,0,1};

	//Setup NED magnetic field unit vector
	float32_t mDip = 10;
	float32_t ctheta = arm_cos_f32(mDip);
	float32_t stheta = arm_sin_f32(mDip);
	float32_t thetaMag = sqrtf(ctheta*ctheta + stheta*stheta);
	float32_t r[3] = {ctheta/thetaMag, 0, stheta/thetaMag};

	//Calculate measurement model matrix h(q). Note this has been massively simplified from the definition using that gx,gy, and ry are 0 in NED
	float32_t h[6] = {
			//Model for gravity
			q[1]*q[3]-q[0]*q[2],
			q[0]*q[1]+q[2]*q[3],
			0.5f-q[1]*q[1]-q[2]*q[2],
			//Model for magnetic field
			r[0]*(0.5f-q[2]*q[2]-q[3]*q[3]) + r[2]*(q[1]*q[3]-q[0]*q[2]),
			r[0]*(q[1]*q[2]-q[0]*q[3]) + r[2]*(q[0]*q[1]+q[2]*q[3]),
			r[0]*(q[0]*q[2]+q[1]*q[3]) + r[2]*(0.5f-q[1]*q[1]-q[2]*q[2])
	};

	//Calculate Jacobian H(q) of measurement model h(q). Note since using NED coordinate, this is massively simplified from the definition
	//H(q) is created by stacking the Jacobians of the measurement models for gravitational acceleration and the magnetic field, both are 3x4
	arm_matrix_instance_f32 H;
	float32_t HData[24] = {
			//Jacobian of model for gravity
			-q[2], q[3],-q[0], q[1],
			 q[1], q[0], q[3], q[2],
			 q[0],-q[1],-q[2], q[3],
			//Jacobian of model for the magnetic field
			 r[0]*q[0]-r[2]*q[2], r[0]*q[1]+r[2]*q[3],-r[0]*q[2]-r[2]*q[0],-r[0]*q[3]+r[2]*q[1],
			-r[0]*q[3]+r[2]*q[1], r[0]*q[2]+r[2]*q[0], r[0]*q[1]+r[2]*q[3],-r[0]*q[0]+r[2]*q[2],
			 r[0]*q[2]+r[2]*q[0], r[0]*q[3]-r[2]*q[1], r[0]*q[0]-r[2]*q[2], r[0]*q[1]+r[2]*q[3]
	};
	arm_mat_init_f32(&H,6,4,HData);
	arm_mat_scale_f32(&H,2.0f,&H);

	//Calculate H^T
	arm_matrix_instance_f32 HT;
	float32_t HTData[24];
	arm_mat_init_f32(&HT,4,6,HTData);
	arm_mat_trans_f32(&H,&HT);

	//Generate noise covariance matrix
	arm_matrix_instance_f32 R;
	float32_t RData[18] = {
			//Accelerometer noise (at 1kHz)
			0.1, 0.0, 0.0,
			0.0, 0.1, 0.0,
			0.0, 0.0, 0.1,
			//Magnetometer noise at 100Hz bandwidth
			0.04,0.00,0.00,
			0.00,0.04,0.00,
			0.00,0.00,0.04
	};
	arm_mat_init_f32(&R,6,3,RData);

	//Calculate the error between the predicted and measured state
	arm_matrix_instance_f32 vt;
	float32_t vtData[6];
	arm_mat_init_f32(&vt,6,1,vtData);

	arm_sub_f32(z,h,vtData,4);

	//Calculate S_t inverse
	arm_matrix_instance_f32 St;
	float32_t StData[18];
	arm_mat_init_f32(&St,6,3,StData);
	arm_mat_mult_f32(&P,&HT,&St);
	arm_mat_mult_f32(&H,&St,&St);
	arm_mat_add_f32(&St,&R,&St);
	arm_mat_inverse_f32(&St,&St);

	//Update the Kalman Gain Matrix
	arm_mat_mult_f32(&HT,&St,&K);
	arm_mat_mult_f32(&P,&K,&K);

	//Correct quaternion
	arm_matrix_instance_f32 Kvt;
	float32_t KvtData[4];
	arm_mat_init_f32(&Kvt,4,1,KvtData);

	arm_mat_mult_f32(&K,&vt,&Kvt);
	arm_mat_add_f32(&Q,&Kvt,&Q);

	//Correct process covariance
	arm_matrix_instance_f32 KH;
	float32_t KHData[16];
	arm_mat_init_f32(&KH,4,4,KHData);

	arm_mat_mult_f32(&K,&H,&KH);

	arm_matrix_instance_f32 I4;
	float32_t I4Data[16] = {
			1.0,0.0,0.0,0.0,
			0.0,1.0,0.0,0.0,
			0.0,0.0,1.0,0.0,
			0.0,0.0,0.0,1.0
	};
	arm_mat_init_f32(&I4,4,4,I4Data);

	arm_mat_sub_f32(&I4,&KH,&KH);

	arm_mat_mult_f32(&KH,&P,&P);

}

void fastqEstimation(ekfState *inst,ICM40609D_Data_t *icmData,MMC5983MA_Data_t *magData) {

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
	inst->q[0] = qw*qmag;
	inst->q[1] = qx*qmag;
	inst->q[2] = qy*qmag;
	inst->q[3] = qz*qmag;
}
