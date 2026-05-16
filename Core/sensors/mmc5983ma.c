#include "mmc5983ma.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

/* Register map */
#define MMC_XOUT0       0x00
#define MMC_TOUT        0x07
#define MMC_STATUS      0x08
#define MMC_CTRL0       0x09
#define MMC_CTRL1       0x0A
#define MMC_CTRL2       0x0B
#define MMC_PRODUCT_ID  0x2F

/* Control bits */
#define MMC_TM_M        0x01  // take magnetic measurement
#define MMC_TM_T        0x02  // take temperature measurement
#define MMC_INT_MD_EN   0x04
#define MMC_SET_BIT     0x08
#define MMC_RESET_BIT   0x10
#define MMC_AUTO_SR_EN  0x20
#define MMC_OTP_LOAD    0x40
#define MMC_SW_RST      0x80

// 18-bit unsigned, center at 2^17 = 131072, full scale ±8 Gauss
#define MMC_SENSITIVITY 16384.0f  // LSB/Gauss

static void MMC_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7F, val }; // bit 7 = 0 for write
    HAL_GPIO_WritePin(MAG_nCS_GPIO_Port, MAG_nCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(MAG_nCS_GPIO_Port, MAG_nCS_Pin, GPIO_PIN_SET);
}

static void MMC_ReadRegs(uint8_t reg, uint8_t *out, uint8_t len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    tx[0] = reg | 0x80; // bit 7 = 1 for read
    HAL_GPIO_WritePin(MAG_nCS_GPIO_Port, MAG_nCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(MAG_nCS_GPIO_Port, MAG_nCS_Pin, GPIO_PIN_SET);
    memcpy(out, &rx[1], len);
}

void MMC5983MA_Init(void)
{
	// Software reset
	MMC_WriteReg(MMC_CTRL1, MMC_SW_RST);
	HAL_Delay(10);

	//Test Product ID
	uint8_t productID;
	MMC_ReadRegs(MMC_PRODUCT_ID,&productID,1);
	if (productID!=0x30) {
		while(1) {printf("MMC Init Error"); HAL_Delay(100);}
	}


    // SET operation to remove residual magnetization
	MMC_WriteReg(MMC_CTRL0, 0x10); // RESET
	HAL_Delay(1);
	MMC_WriteReg(MMC_CTRL0, 0x08); // SET
	HAL_Delay(1);

	MMC_WriteReg(MMC_CTRL1, 0x00);

    // Enable auto set/reset and continuous 100Hz measurement (CTRL2)
    // Bits [2:0] = CM_FREQ: 001 = 1Hz, 010 = 10Hz, 011 = 20Hz,
    //              100 = 50Hz, 101 = 100Hz
    MMC_WriteReg(MMC_CTRL2, 0x05 | 0x10); // 100Hz + CMM_EN (bit 4)
}

void MMC5983MA_Read_All(MMC5983MA_Data_t *data)
{
    // Trigger a single measurement (or rely on continuous mode from Init)
    MMC_WriteReg(MMC_CTRL0, MMC_TM_M );

    // Poll until measurement done (STATUS bit 0 = Meas_M_Done)
    uint8_t status = 0;
    uint32_t timeout = HAL_GetTick() + 10;
    do {
        MMC_ReadRegs(MMC_STATUS, &status, 1);
    } while (!(status & 0x01) && HAL_GetTick() < timeout);

    // Burst read 7 bytes: XOUT0, XOUT1, YOUT0, YOUT1, ZOUT0, ZOUT1, XYZOUT2
    uint8_t raw[7] = {0};
    MMC_ReadRegs(MMC_XOUT0, raw, 7);

    // Reconstruct 18-bit values
    uint32_t x_raw = ((uint32_t)raw[0] << 10) | ((uint32_t)raw[1] << 2) | ((raw[6] >> 6) & 0x03);
    uint32_t y_raw = ((uint32_t)raw[2] << 10) | ((uint32_t)raw[3] << 2) | ((raw[6] >> 4) & 0x03);
    uint32_t z_raw = ((uint32_t)raw[4] << 10) | ((uint32_t)raw[5] << 2) | ((raw[6] >> 2) & 0x03);

    data->x = -((float)x_raw - 131072.0f) / MMC_SENSITIVITY;
    data->y = -((float)y_raw - 131072.0f) / MMC_SENSITIVITY;
    data->z = -((float)z_raw - 131072.0f) / MMC_SENSITIVITY;

    // Read temperature: 1 byte, 0.8°C/LSB, offset at 0 = -75°C
    MMC_WriteReg(MMC_CTRL0, MMC_TM_T);

	// Poll until measurement done (STATUS bit 0 = Meas_M_Done)
	status = 0;
	timeout = HAL_GetTick() + 10;
	do {
		MMC_ReadRegs(MMC_STATUS, &status, 1);
	} while (!(status & 0x02) && HAL_GetTick() < timeout);

    uint8_t t_raw = 0;
    MMC_ReadRegs(MMC_TOUT, &t_raw, 1);
    data->temp = (float)t_raw * 0.8f - 75.0f;
}
