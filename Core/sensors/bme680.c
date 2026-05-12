#include "bme680.h"
#include <string.h>
#include <stdio.h>

static struct bme68x_dev bme;
static uint8_t           dev_addr = BME680_I2C_ADDRESS;
static uint16_t          heatr_dur_ms = 150;


// bme68x fptrs

// note: length cannot be greater than 63 bytes
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr,
                                               const uint8_t *reg_data,
                                               uint32_t length,
                                               void *intf_ptr)
{
    uint8_t dev = *(uint8_t *)intf_ptr;

    uint8_t buf[64];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, length);

    return I2C_transmit(dev, buf, (uint8_t)(length + 1)) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr,
                                              uint8_t *reg_data,
                                              uint32_t length,
                                              void *intf_ptr)
{
    uint8_t dev = *(uint8_t *)intf_ptr;
    uint8_t reg[1] = { reg_addr };

    // note: length is casted to 8 bits, can't be greater than 255
    return I2C_receive(dev, reg, reg_data, 1, (uint8_t)length) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    DWT_Delay_us(period);
}


// public functions


void BME680_Init(void)
{
	uint8_t status = 0;
    bme.read     = bme68x_i2c_read;
    bme.write    = bme68x_i2c_write;
    bme.delay_us = bme68x_delay_us;
    bme.intf     = BME68X_I2C_INTF;
    bme.intf_ptr = &dev_addr;
    bme.amb_temp = 25;

    status = bme68x_init(&bme);
    printf("bme68x_init status: %d\r\n", status);
    struct bme68x_conf conf = {0};
    bme68x_get_conf(&conf, &bme);
    conf.os_temp = BME68X_OS_2X;
    conf.os_pres = BME68X_OS_16X;
    conf.os_hum  = BME68X_OS_1X;
    conf.filter  = BME68X_FILTER_OFF;
    conf.odr     = BME68X_ODR_NONE;
    status = bme68x_set_conf(&conf, &bme);
    printf("bme68x_set_conf status: %d\r\n", status);

    struct bme68x_heatr_conf heatr = {0};
    heatr.enable     = BME68X_ENABLE;
    heatr.heatr_temp = 320;
    heatr.heatr_dur  = heatr_dur_ms;
    status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr, &bme);
    printf("bme_set_heatr_conf status: %d\r\n", status);
}

void BME680_Read_All(BME680_Data_t *data)
{
    struct bme68x_data sensor_data = {0};
    uint8_t n_data = 0;

    bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);

    struct bme68x_conf conf = {0};
    bme68x_get_conf(&conf, &bme);
    uint32_t del_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme)
                      + (uint32_t)(heatr_dur_ms * 1000U);
    DWT_Delay_us(del_us);

    bme68x_get_data(BME68X_FORCED_MODE, &sensor_data, &n_data, &bme);

    if (n_data > 0) {
        data->temperature = sensor_data.temperature;
        data->pressure    = sensor_data.pressure / 100.0f;  // Pa to hPa
        data->humidity    = sensor_data.humidity;
        data->gas         = (float)sensor_data.gas_resistance;
    }
}
