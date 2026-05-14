#ifndef BME680_H
#define BME680_H

#include "main.h"
#include "..\software_i2c\stm32_sw_i2c.h"
#include "..\software_i2c\dwt_stm32_delay.h"
#include "bme68x.h"

#define BME680_I2C_ADDRESS (0x76 << 1)

typedef struct {
    float temperature;
    float pressure;
    float humidity;
    float gas;
    float altitude;
} BME680_Data_t;

typedef struct {
	float h0;
	float t0;
	float p0;
} bmeZeroData;

void BME680_Init(void);
void BME680_Read_All(BME680_Data_t *data);

#endif // BME680_H
