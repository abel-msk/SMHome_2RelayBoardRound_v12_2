/*
 * smhome_sensors.c
 *
 *  Created on: Jul 16, 2023
 *      Author: abel
 */

#include <stdio.h>
#include <string.h>
#include "smhome_sensors.h"
//#include "usart.h"
#include "adc.h"
#include "rtc.h"
#include "smhome_proto.h"

SMH_SensorListElTypeDef* SENSOR_DB = NULL;
//SMH_SensorListElTypeDef* current = 0;
SMH_SensValueReply_t cur_sens_val;
uint16_t* SMH_ConvResultArray = NULL;
uint8_t SMH_ADC_TotChannels = 0;

/**
 *    Initialize in mem sensor DB
 */
void Sensor_DB_Init() {
	SMH_SensorListElTypeDef* cur_el;
	SMH_SensorDescrTypeDef* record;

	SENSOR_DB = (SMH_SensorListElTypeDef*) malloc(sizeof(SMH_SensorListElTypeDef));


	record = Sensor_Init(SW1,"SW1",SENSOR_DIGITAL,SENSOR_IN, 'B', GPIO_PIN_11, ADC_CHANNEL_0);
	SENSOR_DB->el = record;

	record = Sensor_Init(SW2,"SW2",SENSOR_DIGITAL,SENSOR_IN, 'B', GPIO_PIN_10, ADC_CHANNEL_0);
	cur_el = SMH_SensorDB_Add(SENSOR_DB,record);

	record = Sensor_Init(SN1,"SN1",SENSOR_DIGITAL,SENSOR_IN, 'B', GPIO_PIN_2, ADC_CHANNEL_0);
	cur_el = SMH_SensorDB_Add(cur_el,record);

	record = Sensor_Init(SN2,"SN2",SENSOR_DIGITAL,SENSOR_IN, 'B', GPIO_PIN_1, ADC_CHANNEL_9);
	cur_el = SMH_SensorDB_Add(cur_el,record);

	record = Sensor_Init(UTX,"UTX",SENSOR_ANALOG,SENSOR_OUT, 'A', GPIO_PIN_2, ADC_CHANNEL_2);
	cur_el = SMH_SensorDB_Add(cur_el,record);

	record = Sensor_Init(RL1,"RL1",SENSOR_DIGITAL,SENSOR_OUT, 'B', GPIO_PIN_9, ADC_CHANNEL_0);
	cur_el = SMH_SensorDB_Add(cur_el,record);
	record->isLocked = true;

	record= Sensor_Init(RL2,"RL2",SENSOR_DIGITAL,SENSOR_OUT, 'B', GPIO_PIN_8, ADC_CHANNEL_0);
	cur_el = SMH_SensorDB_Add(cur_el,record);
	record->isLocked = true;

	//   PA15 port;  pin  on chip 38
	record = Sensor_Init(ACSENS,"ACSN",SENSOR_ANALOG,SENSOR_IN, 'A', GPIO_PIN_15,ADC_CHANNEL_0);
	record->isLocked = true;
	cur_el = SMH_SensorDB_Add(cur_el, record);


	record = Sensor_Init(TEMP1,"TEMP1",SENSOR_ANALOG,SENSOR_IN, '-', 0, ADC_CHANNEL_TEMPSENSOR);
	record->isLocked = true;
	record->status = SENSOR_UP;
	Sensor_SetPolling(record, 5);
	cur_el = SMH_SensorDB_Add(cur_el, record);

	record = Sensor_Init(VREF,"VREF",SENSOR_ANALOG,SENSOR_IN, '-', 0, ADC_CHANNEL_VREFINT);
	record->isLocked = true;
	record->status = SENSOR_UP;
	cur_el = SMH_SensorDB_Add(cur_el, record);


	//    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	//    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
	//    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
	//    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
	//
	//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	//	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	//	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	//	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/**
 *    Common interrupt handler for all activated PINS. ( Check By DB )
 */
void EXTI15_10_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
}

void EXTI0_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
 *    Final callback for interrupt handling.
 *    Used for sending event when digital input PIN was switched UP/DOWN
 * @param GPIO_Pin
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	SMH_SensorDescrTypeDef* sensor = GetSensorByPinNum(GPIO_Pin);

	if ((sensor != NULL) && ( ! sensor->isAnalog) && sensor->isInput) {

		SMH_SensValueReply_t state;
		state.value = HAL_GPIO_ReadPin(GetPortNum(sensor->pinPort), sensor->pinNum);

		if ((getUpTime() - sensor->switchTime ) > 200 ) {
			SMH_SensorDOThresholds(sensor,state.value==0);    // value==0  toLow, value==1  toHigh

			if (((sensor->isEventOnLow) && (state.value == 0))  ||
					((sensor->isEventOnHigh) && (state.value == 1))) {
				SMHome_SendSensorValue(sensor->id, &state);
			}

			sensor->switchCurState = state.value;
			sensor->switchTime = getUpTime();

		}
	}
}

/**
 *  Convert PORT letter to internal port id
 * @param portName
 * @return
 */
GPIO_TypeDef* GetPortNum(char portName) {

	if (portName == 'A') {
		return GPIOA;
	}
	else if (portName == 'B') {
		return GPIOB;
	}
	else if (portName == 'C') {
		return GPIOC;
	}
	return GPIOB;
}

/**
 * Append sensor record in DB
 * @param last_el
 * @param SensorDescr
 * @return
 */
SMH_SensorListElTypeDef* SMH_SensorDB_Add(SMH_SensorListElTypeDef* last_el, SMH_SensorDescrTypeDef *SensorDescr) {

	SMH_SensorListElTypeDef* el = (SMH_SensorListElTypeDef*) malloc(sizeof(SMH_SensorListElTypeDef));
	last_el->next = el;
	el->el = SensorDescr;
	el->next = NULL;
	return el;
}

/**
record = Sensor_Init(SN1,"SN1",SENSOR_DIGITAL,SENSOR_IN, 'B', GPIO_PIN_2, ADC_CHANNEL_0);
 */

/**
 *
 * @brief Generate initial sensors record
 *
 * @param id
 * @param name
 * @param isAnalog
 * @param Direction
 * @param port
 * @param pin
 * @param channel
 * @return
 */

SMH_SensorDescrTypeDef* Sensor_Init( SensorID_t id, char* name,
		bool isAnalog, bool direction, char port, uint16_t pin, uint32_t channel
)
{

	SMH_SensorDescrTypeDef* sensor = (SMH_SensorDescrTypeDef*) malloc(sizeof(SMH_SensorDescrTypeDef));
	if (sensor == NULL) {
		SMHome_error(RC_NO_MEM);
		return NULL;
	}

	sensor->id = id;
	for ( int i=0; i < SENSOR_NAME_LEN ; i++ ) {
		if ( i >= strlen(name)) {
			sensor->name[i] = ' ';
		}
		else {
			sensor->name[i] = name[i];
		}
	}
	sensor->status = SENSOR_DOWN;
	sensor->isLocked = false;
	sensor->isAnalog = isAnalog;
	sensor->isInput = direction;
	sensor->pinNum = pin;
	sensor->pinPort = port;
	sensor->isExternal = true;
	sensor->isPolling = false;
	sensor->isEventOnLow = false;
	sensor->isEventOnHigh = false;

	sensor->adcChannel = channel;
	sensor->adcRank = 0;

	sensor->switchCurState = 0;
	sensor->switchTime = 0;

	sensor->pollingInterval = 0;
	sensor->lastPollingTime = 0;

	return 	sensor;
}

/**
 * @brief Configure sensor polling interval
 * @param sensor
 * @param period
 */
uint8_t Sensor_SetPolling(SMH_SensorDescrTypeDef* sensor, uint16_t period) {
	uint8_t rc = IS_OK;
	sensor->pollingInterval = period;

	if ( ! sensor->isInput ) {
		return RC_SENSOR_WRONG_TYPE;
	}

	if ( period > 0 )
		sensor->isPolling = true;
	else
		sensor->isPolling = false;

	if ( sensor->isAnalog ) {
		SMH_ADC_RunConversation();
	}
	return rc;
}


/**
 * @brief Configure sensor threshold events
 *
 * @param sensor
 * @param isHigh
 * @param TH_High
 * @param isLow
 * @param TH_Low
 */
uint8_t Sensor_SetEvents(SMH_SensorDescrTypeDef* sensor, bool isHigh, bool isLow) {

	if ( ! sensor->isInput ) {
		return RC_SENSOR_WRONG_TYPE;
	}

	sensor->isEventOnHigh = isHigh;
	sensor->isEventOnLow = isLow;

	return IS_OK;
}


/**
 *     Add record to sensors thresholds list
 * @param sensor
 * @param isToLow
 * @param toSensID
 * @param toSensValue
 * @return
 */

uint8_t Sensor_SetThreshold(SMH_SensorDescrTypeDef* sensor,
		uint8_t ruleId,
		bool isToLow,
		uint16_t thValue,
		SensorID_t sensorId,
		uint16_t sensorValue ) {

	SMH_SensorThreshold_t* threshold = NULL;


	if   ( ! sensor->isInput ) {
		return RC_SENSOR_WRONG_TYPE;
	}

	threshold = (SMH_SensorThreshold_t*) malloc(sizeof(SMH_SensorThreshold_t));
	if (threshold == NULL) {
		return RC_NO_MEM;
	}
	threshold->ruleId = ruleId;
	threshold->next = NULL;
	threshold->isToLow = isToLow;
	threshold->thVaue = thValue;
	threshold->sensorId = sensorId;
	threshold->sensorValue = sensorValue;


	if (sensor->thList == NULL ) {
		sensor->thList = threshold;
	}
	else {
		SMH_SensorThreshold_t* curr = sensor->thList;
		SMH_SensorThreshold_t* lastInList;
		bool isRuleFound = false;

		while (curr != NULL) {
			if ( curr->ruleId == ruleId) {
				curr->isToLow = isToLow;
				curr->thVaue = thValue;
				curr->sensorId = sensorId;
				curr->sensorValue = sensorValue;
				free(threshold);
				threshold = curr;
				isRuleFound = true;
				break;
			}
			lastInList = curr;
			curr = curr->next;
		}
		if ( ! isRuleFound ) {
			lastInList->next = threshold;
		}
	}
	return IS_OK;
}


/**
 *
 * Remove sensor Threshold record (ruleId)  from list
 *
 * @param sensor
 * @param ruleId
 * @return
 */
uint8_t Sensor_ClearThreshold(SMH_SensorDescrTypeDef* sensor, uint8_t ruleId) {

	if (sensor->thList == NULL ) {
		return IS_OK;
	}

	SMH_SensorThreshold_t* curr = sensor->thList;
	SMH_SensorThreshold_t* prev = NULL;

	while (curr != NULL) {
		if (curr->ruleId == ruleId) {
			if (prev == NULL) {              //  if 'prev' no defined it mean we are on first element
				sensor->thList = curr->next;
			}
			else {
				prev->next = curr->next;
			}
			free(curr);
			break;
		}
		prev = curr;
		curr = curr->next;
	}
	return IS_OK;
}


/**
 * Switch sensor OFF
 *
 * @param sensor
 * @return
 */

uint8_t Sensor_OFF(SMH_SensorDescrTypeDef* sensor) {

	//	sensor->SendEvents = SENSOR_EVT_OFF;

	if ( sensor->pinPort != '-') {
		HAL_GPIO_DeInit(GetPortNum(sensor->pinPort), sensor->pinNum);
	}

#ifdef DEBUG_PRINT_UART
	if ( sensor->id == UTX ) {
		HAL_UART_MspInit(&huart2);
	}
#endif

	if ( sensor->isInput) {
		if ( sensor->pinNum == GPIO_PIN_1 ) {
			HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		}
		else if (sensor->pinNum == GPIO_PIN_2 ) {
			HAL_NVIC_DisableIRQ(EXTI2_IRQn);
		}
	}

	sensor->status = SENSOR_DOWN;
	return IS_OK;
}

/**
 * Switch sensor ON
 *
 * @param sensor
 * @return
 */
uint8_t Sensor_ON(SMH_SensorDescrTypeDef* sensor) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if ( sensor->pinPort == '-') {
		sensor->status = SENSOR_UP;
		return IS_OK;
	}

#ifdef DEBUG_PRINT_UART
	if ( sensor->id == UTX ) {
		HAL_UART_MspDeInit(&huart2);   // If we want use UART_TX port for other function, so disable UART
	}
#endif

	/**
	 *   Clear current pin settings
	 */
	HAL_GPIO_DeInit(GetPortNum(sensor->pinPort),sensor->pinPort);
	GPIO_InitStruct.Pin = sensor->pinNum;

	//	if ( sensor->pinNum == GPIO_PIN_1 ) {
	//		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	//	}
	//	else if (sensor->pinNum == GPIO_PIN_2 ) {
	//		HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	//	}

	/**
	 *     Setup ANALOG pin/port settings
	 */
	if (sensor->isAnalog) {
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	}

	/**
	 *     Setup DIGITAL pin/port settings
	 */
	else {
		/**   Digital Input */
		if (sensor->isInput) {
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  // GPIO_MODE_IT_RISING_FALLING | GPIO_MODE_IT_RISING
			GPIO_InitStruct.Pull = GPIO_NOPULL;                  //

			if ( sensor->pinNum == GPIO_PIN_1 ) {
				HAL_NVIC_EnableIRQ(EXTI1_IRQn);
				HAL_NVIC_SetPriority(EXTI1_IRQn, 0x1, 0);
			} else if (sensor->pinNum == GPIO_PIN_2 ) {
				HAL_NVIC_EnableIRQ(EXTI2_IRQn);
				HAL_NVIC_SetPriority(EXTI2_IRQn, 0x1, 0);
			} else if ((sensor->pinNum >= GPIO_PIN_10 ) && (sensor->pinNum <=  GPIO_PIN_15)) {
				HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
				HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x1, 0);
			}
		}
		/**   Digital Output */
		else {
			HAL_GPIO_WritePin(GetPortNum(sensor->pinPort), sensor->pinNum, GPIO_PIN_RESET);
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		}
	}

	HAL_GPIO_Init(GetPortNum(sensor->pinPort), &GPIO_InitStruct);
	sensor->status = SENSOR_UP;
	return IS_OK;
}

/**
 *
 * Search sensor definition by its ID
 *
 * @param sensor id
 * @return
 */
SMH_SensorDescrTypeDef* GetSensorByID(SensorID_t id) {

	SMH_SensorListElTypeDef* cur = SENSOR_DB;

	while  (cur != 0) {
		if ( cur->el->id == id ) {
			return cur->el;
		}
		cur = cur->next;
	}
	return NULL;
}

/**
 *
 *   Select sensor by pin number
 *
 * @param sensors pin
 * @return
 */
SMH_SensorDescrTypeDef* GetSensorByPinNum(uint16_t pin) {
	SMH_SensorListElTypeDef* cur = SENSOR_DB;
	while  (cur != 0) {
		if ( cur->el->pinNum == pin ) {
			if (cur->el->pinPort == 'B' ) {
				return cur->el;
			}
		}
		cur = cur->next;
	}
	return NULL;
}

/**
 *
 *    Configure  ADC Scan channels based on sensors params
 *
 */
void SMH_ADC_RunConversation() {
	SMH_SensorListElTypeDef* cur = SENSOR_DB;
	SMH_SensorDescrTypeDef* sensor;
	ADC_ChannelConfTypeDef sConfig = {0};
	uint8_t ch_total_count = 0;

	if ( SMH_ConvResultArray != NULL) {
		free(SMH_ConvResultArray);
	}

	//	HAL_ADC_MspDeInit(&hadc1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_DeInit(&hadc1);

	/** Calculate amount of conversion channels
	 */
	while  (cur != 0) {
		sensor = cur->el;
		if (sensor->isAnalog && sensor->isInput  &&
				(sensor->adcChannel != ADC_CHANNEL_0) &&
				(sensor->status = SENSOR_UP)
		){
			ch_total_count++;
		}
		cur = cur->next;
	}

	/** Configure ADC1
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = ch_total_count;

	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		SMHome_SendError(NULL, NULL, RC_SENSOR_CHCONF_ERR);
	}

	/** Configure Channels
	 */
	cur = SENSOR_DB;
	uint8_t cur_adc_rank = 1;

	while  (cur != 0) {
		sensor = cur->el;
		if ((sensor->isAnalog) && sensor->isInput &&
				(sensor->adcChannel != ADC_CHANNEL_0) &&
				(sensor->status = SENSOR_UP)
		){

			sConfig.Channel = sensor->adcChannel;
			sConfig.Rank = cur_adc_rank;
			sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;

			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK) {
				sensor->adcRank = cur_adc_rank;
				cur_adc_rank++;
			}
			else {
				SMHome_SendError(NULL, NULL, RC_SENSOR_CHCONF_ERR);
			}
		}
		cur = cur->next;
	}

	SMH_ADC_TotChannels = cur_adc_rank - 1;

	/** Prepare buffer for store result and start DMS conversation save
	 */

	SMH_ConvResultArray = (uint16_t*) malloc(sizeof(uint16_t) * SMH_ADC_TotChannels);
	if ( SMH_ConvResultArray == NULL) {
		SMHome_SendError(NULL, NULL, RC_NO_MEM);
	}
	else {
		//		HAL_ADC_MspInit(&hadc1);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)SMH_ConvResultArray,SMH_ADC_TotChannels);
	}
}

/**
 *
 *  Loop through all polled sensors, get value end send over CAN
 *  Called from main loop.
 *
 */
void SMH_SensorDOPolling() {

	SMH_SensorListElTypeDef* sensor_ptr = SENSOR_DB;
	SMH_SensorDescrTypeDef* sensor;
	SMH_SensorDescrTypeDef* vref_sensor = GetSensorByID(VREF);

	while  (sensor_ptr != 0) {
		sensor = sensor_ptr->el;
		if ( (sensor->status == SENSOR_ON) &&
				sensor->isInput && sensor->isPolling &&
				(sensor->pollingInterval > 0) ) {

			sensor->lastPollingTime++;
			if (sensor->lastPollingTime > sensor->pollingInterval ) {

				/**
				 *   Time to send polled value for this sensor.
				 */
				SMH_SensValueReply_t polled = {0,0,0};
				if ( sensor->isAnalog ) {
					if (vref_sensor->adcRank > 0)
						polled.vref = SMH_ConvResultArray[vref_sensor->adcRank-1];

					if (sensor->adcRank > 0) {
						polled.value = SMH_ConvResultArray[sensor->adcRank-1];
					}
				}
				else {
					polled.value = HAL_GPIO_ReadPin(GetPortNum(sensor->pinPort), sensor->pinNum);
				}

				SMHome_SendSensorValue(sensor->id, &polled);
				sensor->lastPollingTime=0;
			}
		}
		sensor_ptr = sensor_ptr->next;
	}
}

/**
 *
 *  On sensor Event  check  thresholds and perform action on connected sensor (if exist)
 *
 * @param sensor
 * @param changeDirection
 * @return
 */
uint8_t SMH_SensorDOThresholds(SMH_SensorDescrTypeDef* sensor, bool changeDirection) {

	SMH_SensorThreshold_t *thCurrent = sensor->thList;

	while ( thCurrent != NULL) {
		if ((thCurrent->sensorId > BOARD) && (thCurrent->sensorId < VREF) && (thCurrent->isToLow == changeDirection)) {

			SMH_SensorDescrTypeDef*  toSensor = GetSensorByID(thCurrent->sensorId);
			if ((toSensor != NULL) && ( ! toSensor->isInput)) {
				if ( toSensor->isAnalog ) {
					// TODO: DO Threshold for analog sensor.
				}
				else{
					SMH_SensorSwitch(toSensor,thCurrent->sensorValue);
				}
			}
		}
	}
	return IS_OK;
}




// https://electronics.stackexchange.com/questions/324321/reading-internal-temperature-sensor-stm32
// https://deepbluembedded.com/stm32-adc-read-example-dma-interrupt-polling/
/**
 *
 * Get sensor value for analog or digital input
 *
 * @param sensor
 * @return
 */
SMH_SensValueReply_t* SMH_SensorGetValue(SMH_SensorDescrTypeDef* sensor) {

	cur_sens_val.value = 0;
	cur_sens_val.vref = 0;
	cur_sens_val.power_of_ten = 0;

	SMH_SensorDescrTypeDef* vref_sensor = GetSensorByID(VREF);

	if (sensor->isInput && (sensor->status = SENSOR_UP)) {
		if (( sensor->isAnalog ) && (sensor->adcChannel != ADC_CHANNEL_0 )) {

			if (vref_sensor->adcRank > 0)
				cur_sens_val.vref = SMH_ConvResultArray[vref_sensor->adcRank-1];
			if (sensor->adcRank > 0)
				cur_sens_val.value = SMH_ConvResultArray[sensor->adcRank-1];

			//			SMHome_SendSensorValue(sensor->id, &polled);


#ifdef TMPSENSOR_CALC_INTERNAL
			if (sensor->id == TEMP1 ) {
				cur_sens_val.value = (uint16_t) TMPSENSOR_getTemperature(cur_sens_val.value,cur_sens_val.vref) * 100;
				cur_sens_val.power_of_ten = -2;
			}
#endif
		}
		else if(( ! sensor->isAnalog ) ) {
			cur_sens_val.value = HAL_GPIO_ReadPin(GetPortNum(sensor->pinPort), sensor->pinNum);
		}
		else if (sensor->adcChannel == ADC_CHANNEL_0) {
			SMHome_SendError(NULL, NULL, RC_ADC_CH_UNUSED);
		}
		else if (sensor->status == SENSOR_DOWN) {
			SMHome_SendError(NULL, NULL, RC_ADC_CH_DOWN);
		}
	}
	return &cur_sens_val;
}

/**
 *
 * Switch sensor state 1/0  for digital output sensor
 *
 * @param sensor
 * @return
 */
uint8_t SMH_SensorSwitch(SMH_SensorDescrTypeDef* sensor, uint8_t value) {

	if (! sensor->isInput ) {
		if (! sensor->isAnalog) {
			if ( value == 1 ) {
				HAL_GPIO_WritePin(GetPortNum(sensor->pinPort), sensor->pinNum, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(GetPortNum(sensor->pinPort), sensor->pinNum, GPIO_PIN_RESET);
			}
		} else  {
			// Set analog value
		}
	}
	return IS_OK;
}


/**
 * @brief Calculate temperature (tested on STM32F401, other MCU may have different constants!)
 * @note If IntRef not use, set it [ex.: #define TMPSENSOR_USE_INTREF 0]
 * @param Temperature sensor's ADC 16-bit value, Internal Reference ADC 16-bit value (if use)
 * @retval Internal sensor temperature
 */
#ifdef TMPSENSOR_CALC_INTERNAL

double TMPSENSOR_getTemperature(uint16_t adc_sensor, uint16_t adc_intref) {
	double intref_vol = (TMPSENSOR_ADCMAX*TMPSENSOR_ADCVREFINT)/adc_intref;
	//	double sensor_vol = adc_sensor * intref_vol/TMPSENSOR_ADCMAX;
	return ((adc_sensor * intref_vol/TMPSENSOR_ADCMAX) - TMPSENSOR_V25)/
			((TMPSENSOR_AVGSLOPE)*1000)+25;
}

#endif

