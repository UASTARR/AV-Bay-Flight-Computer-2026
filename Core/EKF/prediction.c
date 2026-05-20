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

}

void predictAttitude(ekfState *inst,float32_t dt,ICM40609D_Data_t *icmData) {
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

	//Init sum matrix
	arm_matrix_instance_f32 sum;
	float32_t sumData[16];
	arm_mat_init_f32(&sum,4,4,sumData);

	//Calculate 2nd order Taylor polynomial
	arm_mat_sum(&I,&omega,&sum); //Add first term
	arm_mat_scale(&omega,0.5,&omega); //Multiply omega by 1/2! for second term
	arm_mat_sum(&sum,&omega,&sum); //Add new omega to sum

	//Update state
	arm_mat_vec_mult_f32(&sum,inst->qold,inst->q); //Multiply the sum by the previous state
}
