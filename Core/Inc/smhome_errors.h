/*
 * project_errors.h
 *
 *  Created on: Oct 14, 2023
 *      Author: abel
 */

#ifndef INC_SMHOME_ERRORS_H_
#define INC_SMHOME_ERRORS_H_

#include "main.h"

#ifdef DEBUG_PRINT_UART
#include <stdio.h>
#include "usart.h"
#endif

#define SENSOR_NOT_FOUND 108

#define RC_SENSOR_NOT_FOUND 100
#define RC_SENSOR_WRONG_TYPE 101
#define RC_SENSOR_INCORRECT_PARAM 102
#define RC_SENSOR_CONFIG_ERR 103


#define RC_CAN_UNKNOWN_CMD 104
#define RC_CAN_TRANSMIT_ERR 105
#define RC_NO_MEM 106

#define RC_SENSOR_OFF_ERR 107
#define RC_SENSOR_CHCONF_ERR 109
#define RC_ADC_VALUE_ERR 110
#define RC_ADC_CH_UNUSED 111
#define RC_ADC_CH_DOWN 112



void SMHome_error(int);

#endif /* INC_SMHOME_ERRORS_H_ */
