#include "ms5611.h"
#include <math.h>

static uint16_t prom[8];

void MS5611_Init(void) {
    // send reset command
    uint8_t reset_cmd[1] = { 0x1E };
    I2C_transmit(MS5611_I2C_ADDRESS, reset_cmd, 1);
    DWT_Delay_us(10000);

    // read PROM calibration coefficients
    for (int i = 0; i < 6; i++) {
        uint8_t prom_cmd[1] = { (uint8_t)(0xA2 + i * 2) };
        uint8_t buf[2] = {0};
        I2C_receive(MS5611_I2C_ADDRESS, prom_cmd, buf, 1, 2);
        prom[i + 1] = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return;
}

void MS5611_Read_All(MS5611_Data_t *data) {
    uint8_t buf[3] = {0};

    uint8_t d1_cmd[1] = { 0x48 };
    I2C_transmit(MS5611_I2C_ADDRESS, d1_cmd, 1);
    DWT_Delay_us(10000);

    uint8_t adc_cmd[1] = { 0x00 };
    I2C_receive(MS5611_I2C_ADDRESS, adc_cmd, buf, 1, 3);
    uint32_t D1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

    uint8_t d2_cmd[1] = { 0x58 };
    I2C_transmit(MS5611_I2C_ADDRESS, d2_cmd, 1);
    DWT_Delay_us(10000);

    I2C_receive(MS5611_I2C_ADDRESS, adc_cmd, buf, 1, 3);
    uint32_t D2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

    int32_t dT   = (int32_t)D2 - ((int32_t)prom[5] << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * prom[6]) >> 23);

    int64_t OFF  = ((int64_t)prom[2] << 16) + (((int64_t)prom[4] * dT) >> 7);
    int64_t SENS = ((int64_t)prom[1] << 15) + (((int64_t)prom[3] * dT) >> 8);
    int32_t P    = (int32_t)(((D1 * SENS >> 21) - OFF) >> 15);

    data->pressure = P / 100.0f; // mbar
    data->altitude = 44330.0f * (1.0f - powf(data->pressure / 1013.25f, 0.1903f));
}
