/*
 * state.c
 *
 *  Created on: Aug 16, 2026
 *      Author: alexk
 */

#include "state.h"
#include "arm_math.h"

static int LIFTOFF_THRESH = 5;
static float BURNOUT_THRESH = 1.5;
static float DBARO_THRESH = -5;
static float DECENT_VELO_THRESH = -5;
static int BURNOUT_TIMER = 100;
static int APO_TIMER = 20;

void stateController(stateParams *state) {
	switch (state->flightState) {
		case (0):
		{
			state->R=1000;
			state->K=0.000003;
			state->Y=0;
			state->T=0.1;
			state->update = 0;

			if (state->accel[3] > LIFTOFF_THRESH) {
				state->flightState=1;
			}
		}
		case (1):
		{
			state->R=1000;
			state->K=0.000003;
			state->Y=0;
			state->T=0.1;
			state->update = 0;

			if (fabsf((float32_t) state->vaccel) < BURNOUT_THRESH)
			{
				state->burnoutCounter +=1;
				if (state->burnoutCounter > BURNOUT_TIMER)
				{
					state->flightState = 2;
				}
			}
			else {
				state->burnoutCounter = 0;
			}
		}
		case (2):
		{
			state->R=0.15;
			state->K=0.003;
			state->Y=0.2;
			state->T=0.8;
			if ((state->dAlt<DBARO_THRESH || state->vaccel<DECENT_VELO_THRESH) && state->alt>100)
			{
				state->apoCounter += 1;
				if (state->apoCounter > APO_TIMER)
				{
					state->flightState = 3;
				}
			else {
				state->apoCounter = 0;
			}
		}
	}
}
}

