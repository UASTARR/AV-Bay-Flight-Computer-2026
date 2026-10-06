/*
 * state.h
 *
 *  Created on: Aug 16, 2026
 *      Author: alexk
 */

#ifndef STATE_MACHINE_STATE_H_
#define STATE_MACHINE_STATE_H_

#include "..\EKF\ekf.h"

typedef struct {
	int flightState;
	float R;
	float K;
	float Y; //Was P
	float T; //Was Q
	int update;
	float alt;
	float velo;
	float accel[3];
	float vaccel;
	float baro;
	float dAlt;
	int burnoutCounter;
	int apoCounter;
	float32_t Q[4];
	float32_t P[4];
} stateParams;

void stateController(stateParams *state);

#endif /* STATE_MACHINE_STATE_H_ */
