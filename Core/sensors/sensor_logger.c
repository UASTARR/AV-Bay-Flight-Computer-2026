/*
 * sensor_logger.c
 *
 *  Created on: Aug 18, 2026
 *      Author: alexk
 */
#include "sensor_logger.h"
#include "FatFs.h"
#include <stdio.h>

FRESULT logSensors(FATFS *FatFs, FIL *fil, char *writeData, UINT *bytesWrote) {
	// 1. Mount the SD Card filesystem
	FRESULT fres;
	fres = f_mount(FatFs, "", 1); // 1 = Mount immediately
	if (fres != FR_OK) {
	    // Handle error (e.g., flash an LED or print to UART)
	    printf("f_mount failed! Error code: %d\r\n", fres);
	} else {

	    // 2. Open or create the file
	    // FA_WRITE: Request write access
	    // FA_OPEN_ALWAYS: Opens the file if it exists, creates it if it doesn't
	    fres = f_open(fil, "log.txt", FA_WRITE | FA_OPEN_ALWAYS);
	    if (fres != FR_OK) {
	        printf("f_open failed! Error code: %d\r\n", fres);
	    } else {

	        // 3. Move the file pointer to the end of the file (Optional: for appending data)
	        f_lseek(fil, f_size(fil));

	        // 4. Write data to the file
	        fres = f_write(fil, writeData, sizeof(writeData) - 1, bytesWrote);
	        if (fres != FR_OK) {
	            printf("f_write failed! Error code: %d\r\n", fres);
	        }

	        // 5. Close the file (CRITICAL: Saves changes from internal RAM buffer to physical SD Card)
	        f_close(&fil);
	    }
	}
    return fres;
}
