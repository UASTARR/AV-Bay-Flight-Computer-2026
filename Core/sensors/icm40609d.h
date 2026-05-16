#ifndef ICM40609D_H
#define ICM40609D_H

#include "main.h"

typedef struct {
    float accel_x;  // g
    float accel_y;  // g
    float accel_z;  // g
    float gyro_x;   // deg/s
    float gyro_y;   // deg/s
    float gyro_z;   // deg/s
    float temp;
} ICM40609D_Data_t;

typedef struct {
	_Bool xsign;
	_Bool ysign;
	_Bool zsign;
	int xindex;
	int yindex;
	int zindex;
} imuZeroData;


void ICM40609D_Init(void);
void ICM40609D_Read_All(ICM40609D_Data_t *data);
void ICM40609D_Zero(void);

#endif // ICM40609D_H
