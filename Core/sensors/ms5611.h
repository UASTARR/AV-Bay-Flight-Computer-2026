#ifndef MS5611_H
#define MS5611_H

#include "main.h"
#include "..\software_i2c\stm32_sw_i2c.h"
#include "..\software_i2c\dwt_stm32_delay.h"

#define MS5611_I2C_ADDRESS (0x77 << 1)

typedef struct {
    float pressure;
    float altitude;
} MS5611_Data_t;

void MS5611_Init(void);
void MS5611_Read_All(MS5611_Data_t *data);

#endif // MS5611_H
