/*
 * sensor_logger.h
 *
 *  Created on: Aug 18, 2026
 *      Author: alexk
 */

#ifndef SENSORS_SENSOR_LOGGER_H_
#define SENSORS_SENSOR_LOGGER_H_

#include "FatFs.h"

FRESULT logSensors(FATFS *FatFs, FIL *fil, char *writeData, UINT *bytesWrote);


#endif /* SENSORS_SENSOR_LOGGER_H_ */
