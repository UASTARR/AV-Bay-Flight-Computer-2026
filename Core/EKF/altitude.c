/*
 * altitude.c
 *
 *  Created on: Aug 16, 2026
 *      Author: alexk
 */

#include "..\State_Machine\state.h"
#include "arm_math.h"
#include "altitude.h"


void altFilter (stateParams *state,float dt) {
	arm_matrix_instance_f32 F;
	float32_t f[4] = {1.0, dt, 0.0, 1.0};
	arm_mat_init_f32(&F,2,2,f);

	arm_matrix_instance_f32 B;
	float32_t b[2] = {0.5*powf(dt,2), dt};
	arm_mat_init_f32(&B,2,1,b);

	float32_t GtoFts2 = 32.174;

	arm_matrix_instance_f32 Q;
	float32_t q[4] = {
			0.25*powf(dt,4)*GtoFts2*state->K, 0.5*powf(dt,3)*GtoFts2*state->K,
			0.5*powf(dt,3)*GtoFts2*state->K, powf(dt,2)*GtoFts2*state->K
	};
	arm_mat_init_f32(&Q,2,2,q);

	state->alt = state->alt + state->velo*dt + 32.174*0.5*state->vaccel*powf(dt,2);
	state->velo = state->velo + 32.174*state->vaccel*dt;

	arm_matrix_instance_f32 P;
	arm_mat_init_f32(&P,2,2,state->P);

	arm_mat_mult_f32(&F,&P,&P);
	arm_mat_trans_f32(&F,&F);
	arm_mat_mult_f32(&P,&F,&P);
	arm_mat_add_f32(&P,&Q,&P);
	if (state->update!=1){
		return;
	}
	state->update=0;

	arm_matrix_instance_f32 I;
	float32_t i[4] ={
			1.0,0,
			0,1.0
	};
	arm_mat_init_f32(&I,2,2,i);

	float32_t v = state->baro - state->alt;
	float32_t S = state->P[0] + state->R;
	float32_t K[2];
	K[0] = state->P[0]*1/S;
	K[1] = state->P[2]*1/S;
	state->alt = state->alt + K[0]*v;

	arm_matrix_instance_f32 O;
	float32_t o[4] = {
			K[0],0,
			K[1],0,
	};
	arm_mat_init_f32(&O,2,2,o);
	arm_mat_sub_f32(&I,&O,&I);
	arm_mat_mult_f32(&I,&P,&P);
	return;
}
