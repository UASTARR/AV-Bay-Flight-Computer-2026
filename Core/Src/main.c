/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "..\sensors\bme680.h"
#include "..\sensors\ms5611.h"
#include "..\sensors\icm40609d.h"
#include "..\sensors\mmc5983ma.h"
#include "..\software_i2c\dwt_stm32_delay.h"
#include "..\software_i2c\stm32_sw_i2c.h"
#include "..\attitude_estimation\attitude_estimation.h"
#include "..\EKF\prediction.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Delay_Init();
  I2C_init();
  DWT_Delay_us(1000); //1 ms
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  //MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  BME680_Init();
  MS5611_Init();
  ICM40609D_Init();
  MMC5983MA_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    BME680_Data_t   bme  = {0};
	    MS5611_Data_t   ms   = {0};
	    ICM40609D_Data_t icm = {0};
	    MMC5983MA_Data_t mag = {0};
	    float32_t q[4] = {0};

	    BME680_Read_All(&bme);
	    MS5611_Read_All(&ms);
	    ICM40609D_Read_All(&icm);
	    MMC5983MA_Read_All(&mag);
	    //AttitudeEst(&ang,&icm,&mag);
	    qernionAttitudeEst(&q,&icm,&mag);

//	    printf("--- BME680 ---\r\n");
//	    printf(">Temp:%.2f\r\n", bme.temperature);
//	    printf("  Press: %.2f hPa\r\n", bme.pressure);
//	    printf("  Hum:   %.2f %%\r\n", bme.humidity);
//	    printf("  Gas:   %.0f ohm\r\n", bme.gas);
//	    printf("  Alt:   %.2f m\r\n", bme.altitude);
//
//	    printf("--- MS5611 ---\r\n");
//	    printf("  Press: %.2f hPa\r\n", ms.pressure);
//	    printf("  Alt:   %.2f m\r\n",   ms.altitude);
//
//	    printf("--- ICM-40609D ---\r\n");
	    printf(">ax:%.3f\r\n>ay:%.3f\r\n>az:%.3f\r\n",   icm.accel_x, icm.accel_y, icm.accel_z);
//	    printf("  Gyro:  %.2f  %.2f  %.2f dps\r\n", icm.gyro_x,  icm.gyro_y,  icm.gyro_z);
//	    printf("  Temp:  %.1f C\r\n", icm.temp);
//
//	    printf("--- MMC5983MA ---\r\n");
	    printf(">mx:%.4f\r\n>my:%.4f\r\n>mz:%.4f\r\n", mag.x, mag.y, mag.z);
	    float bearing = atan2(mag.y,-mag.x)*180/3.14;
	    bearing = (bearing>=0)?bearing:bearing+360;
	    printf(">bearing:%.4f\r\n",bearing);
//	    printf("  Temp:  %.1f C\r\n\r\n", mag.temp);

//	    printf("--- Attitude Estimation ---\r\n");
//	    printf(">3D|my_super_cube:S:cube:W:1:D:0.7:H:0.1:C:blue:Q:%.4f:%.4f:%.4f:%.4f\r\n", q.qw, -q.qy, -q.qz, -q.qx);
	    printf(">3D|my_super_cube:S:cube:W:1:D:0.7:H:0.1:C:blue:Q:%.4f:%.4f:%.4f:%.4f\r\n", q[1], q[3], q[2], q[0]);
//		printf("Angles:%.4f  %.4f  %.4f\r\n", ang.x, ang.y, ang.z);
	    HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// redirect printf to CDC / virtual com port
int _write(int file, char *ptr, int len)
{
//   for usb printf
   CDC_Transmit_FS((uint8_t *)ptr, (uint16_t)len);
   HAL_Delay(1);
   return len;

  // for uart/stlink printf
//  HAL_UART_Transmit(&huart4, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
//  return len;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
