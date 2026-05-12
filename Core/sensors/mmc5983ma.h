#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "main.h"

typedef struct {
    float x; // gauss
    float y;
    float z;
    float temp;
} MMC5983MA_Data_t;

void MMC5983MA_Init(void);
void MMC5983MA_Read_All(MMC5983MA_Data_t *data);

#endif // MMC5983MA_H