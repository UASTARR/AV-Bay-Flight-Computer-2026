/*
 * channel_control.c
 *
 *  Created on: Aug 6, 2026
 *      Author: alexk
 */

#include <stdio.h>
#include "main.h"

void fire_channel(int sel) {
	HAL_GPIO_WritePin(GPIOC, FC_3_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, FC_3_Pin, GPIO_PIN_RESET);
	return;
}
