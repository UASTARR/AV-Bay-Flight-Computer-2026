#include "icm40609d.h"
#include <stdio.h>
#include <math.h>

extern SPI_HandleTypeDef hspi1;

#define ICM_REG_DEVICE_CONFIG  0x11
#define ICM_REG_PWR_MGMT0      0x4E
#define ICM_REG_GYRO_CONFIG0   0x4F
#define ICM_REG_ACCEL_CONFIG0  0x50
#define ICM_REG_TEMP_DATA1     0x1D
#define ICM_REG_WHO_AM_I       0x75

#define ICM_WHO_AM_I_VAL       0x3B

#define ICM_ACCEL_SENS  2048.0f
#define ICM_GYRO_SENS   16.4f

static void ICM_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7F, val };
    HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
}

static void ICM_ReadRegs(uint8_t reg, uint8_t *out, uint8_t len)
{
//    uint8_t tx[16] = {0};
//    uint8_t rx[16] = {0};
//    tx[0] = reg | 0x80;
//    HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 1, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
//    memcpy(out, &rx[1], len);
	uint8_t addr = reg | 0x80;

	HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);

	// 1. Send the address byte
	HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);

	// 2. Receive the 'len' bytes of data
	// The sensor will now shift out data while the MCU sends dummy clocks
	HAL_SPI_Receive(&hspi1, out, len, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
}

void ICM40609D_Init(void)
{
    ICM_WriteReg(ICM_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(2);

    uint8_t who = 0;
    ICM_ReadRegs(ICM_REG_WHO_AM_I, &who, 1);
    if (who != ICM_WHO_AM_I_VAL) {
    	printf("ERROR: ICM40609 WHOAMI=%d\r\n", who);
    }

    ICM_WriteReg(ICM_REG_GYRO_CONFIG0, (0x00 << 5) | (0x000 << 3) | 0x06);

    ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, (0x01 << 5) | 0x06);

    ICM_WriteReg(ICM_REG_PWR_MGMT0, 0x0F);
    HAL_Delay(10);
}

void ICM40609D_Read_All(ICM40609D_Data_t *data)
{
    uint8_t raw[14] = {0};
    ICM_ReadRegs(ICM_REG_TEMP_DATA1, raw, 14);

    int16_t temp_raw  = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t accel_x   = ((int16_t)((raw[4]  << 8) | raw[5])); //default index 3
    int16_t accel_y   = ((int16_t)((raw[2]  << 8) | raw[3])); //default index 5
    int16_t accel_z   = -((int16_t)((raw[6]  << 8) | raw[7])); //default index 7
    int16_t gyro_x    = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gyro_y    = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gyro_z    = (int16_t)((raw[12] << 8) | raw[13]);

    data->temp    = (float)temp_raw  / 132.48f + 25.0f;
    data->accel_x = (float)accel_x  / ICM_ACCEL_SENS;
    data->accel_y = (float)accel_y  / ICM_ACCEL_SENS;
    data->accel_z = (float)accel_z  / ICM_ACCEL_SENS;
    data->gyro_x  = (float)gyro_x   / ICM_GYRO_SENS;
    data->gyro_y  = (float)gyro_y   / ICM_GYRO_SENS;
    data->gyro_z  = (float)gyro_z   / ICM_GYRO_SENS;
}
