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

	//Init W
	arm_matrix_instance_f32 W;
	float32_t WData[12] ={
			-inst->q[1],-inst->q[2],-inst->q[3],
			 inst->q[0],-inst->q[3], inst->q[2],
			 inst->q[3], inst->q[0],-inst->q[1],
			-inst->q[2], inst->q[1], inst->q[0]
	};
	arm_mat_init_f32(&W,4,3,Wdata);
	arm_mat_scale_f32(&W,dt/2,&W); //Scale by dt/2

	arm_matrix_instance_f32 sigma;
		//Add more here once we figure out wtf this is

	//Compute Q = W * sigma * W^T
	arm_mat_mult_f32(&W,&sigma,&Q);
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
