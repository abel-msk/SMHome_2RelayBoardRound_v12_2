/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef DEBUG_PRINT_UART
	#include "stdio.h"
#endif
#include "can_rx_queue.h"
#include <smhome_errors.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern bool doPolling;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//#define TMPSENSOR_CALC_INTERNAL 1

#define TMPSENSOR_AVGSLOPE  4.3    /* mV/°C */
#define TMPSENSOR_V25  1.43        /* V (at 25 °C)  */
#define TMPSENSOR_ADCMAX 4095.0    /* 12-bit ADC maximum value (12^2)-1)  */
#define TMPSENSOR_ADCREFVOL  3.3   /* Typical reference voltage, V  */
#define TMPSENSOR_ADCVREFINT  1.21  /* Internal reference voltage, V  */

//#define TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS 4.3f
//#define TEMP_SENSOR_VOLTAGE_MV_AT_25 1430.0f
//
//#define ADC_REFERENCE_VOLTAGE_MV  1210.0f
//#define ADC_MAX_OUTPUT_VALUE  4095.0f


#define LOG_SERIAL 1
#define LOG_DEBUG  2
#define LOG_NONE   0
//#define DEBUG_PRINT_NUCLEO
//#define DEBUG_PRINT_UART


#define IS_ERROR 1
#define IS_OK  0

#define CAN_MASTERS_NUM 2



/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#ifdef DEBUG_PRINT_UART

#define println(...)  do { printf("[LOG %s:%d] ", __FILE_NAME__,__LINE__);\
	printf(__VA_ARGS__); \
	printf("\r\n");\
	} while (0)

#define log_err(...)  do { printf("[LOG %s:%d] ", __FILE_NAME__,__LINE__);\
	printf(__VA_ARGS__); \
	printf("\r\n");\
	} while (0)

#endif

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')



/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW2_Pin GPIO_PIN_10
#define SW2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_11
#define SW1_GPIO_Port GPIOB
#define ADDR_B8_Pin GPIO_PIN_12
#define ADDR_B8_GPIO_Port GPIOB
#define ADDR_B7_Pin GPIO_PIN_13
#define ADDR_B7_GPIO_Port GPIOB
#define ADDR_B6_Pin GPIO_PIN_14
#define ADDR_B6_GPIO_Port GPIOB
#define ADDR_B5_Pin GPIO_PIN_15
#define ADDR_B5_GPIO_Port GPIOB
#define ADDR_B4_Pin GPIO_PIN_8
#define ADDR_B4_GPIO_Port GPIOA
#define ADDR_B3_Pin GPIO_PIN_9
#define ADDR_B3_GPIO_Port GPIOA
#define ADDR_B2_Pin GPIO_PIN_10
#define ADDR_B2_GPIO_Port GPIOA
#define ADDR_B1_Pin GPIO_PIN_5
#define ADDR_B1_GPIO_Port GPIOB
#define SIG_RL2_Pin GPIO_PIN_8
#define SIG_RL2_GPIO_Port GPIOB
#define SIG_RL1_Pin GPIO_PIN_9
#define SIG_RL1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
