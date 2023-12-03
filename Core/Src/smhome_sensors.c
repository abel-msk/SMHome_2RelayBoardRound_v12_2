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


	record = Sensor_Init(SW1,"SW1",SENSOR_DIGITAL,SENSOR_IN,SENSOR_EXT,SENSOR_EVT_OFF,SMH_SENS_TYPE_UNKNOWN,0,0,GPIO_PIN_11,'B', ADC_CHANNEL_0);
	SENSOR_DB->el = record;

	record = Sensor_Init(SW2,"SW2",SENSOR_DIGITAL,SENSOR_IN,SENSOR_EXT,SENSOR_EVT_OFF,SMH_SENS_TYPE_UNKNOWN,0,0,GPIO_PIN_10,'B', ADC_CHANNEL_0);
	cur_el = SMH_SensorDB_Add(SENSOR_DB,record);

	record = Sensor_Init(SN1,"SN1",SENSOR_DIGITAL,SENSOR_IN,SENSOR_EXT,SENSOR_EVT_OFF,SMH_SENS_TYPE_UNKNOWN,0,0,GPIO_PIN_2,'B', ADC_CHANNEL_0);
	cur_el = SMH_SensorDB_Add(cur_el,record);

	record = Sensor_Init(SN2,"SN2",SENSOR_DIGITAL,SENSOR_IN,SENSOR_EXT,SENSOR_EVT_OFF,SMH_SENS_TYPE_UNKNOWN,0,0,GPIO_PIN_1,'B', ADC_CHANNEL_9);
	cur_el = SMH_SensorDB_Add(cur_el,record);

	record = Sensor_Init(UTX,"UTX",SENSOR_ANALOG,SENSOR_OUT,SENSOR_EXT,SENSOR_EVT_OFF,SMH_SENS_TYPE_UNKNOWN,0,0,GPIO_PIN_2,'A', ADC_CHANNEL_2);
	cur_el = SMH_SensorDB_Add(cur_el,record);

	record = Sensor_Init(RL1,"RL1",SENSOR_DIGITAL,SENSOR_OUT,SENSOR_EXT,SENSOR_EVT_OFF,SMH_SENS_TYPE_RELAY,0,0,GPIO_PIN_9,'B', ADC_CHANNEL_0);
	cur_el = SMH_SensorDB_Add(cur_el,record);
	record->Locked = true;

	record= Sensor_Init(RL2,"RL2",SENSOR_DIGITAL,SENSOR_OUT,SENSOR_EXT,SENSOR_EVT_OFF,SMH_SENS_TYPE_RELAY,0,0,GPIO_PIN_8,'B', ADC_CHANNEL_0);
	cur_el = SMH_SensorDB_Add(cur_el,record);
	record->Locked = true;

	//   PA15 port;  pin  on chip 38
	record = Sensor_Init(ACSENS,"ACSN",SENSOR_ANALOG,SENSOR_IN,SENSOR_EXT,SENSOR_EVT_OFF,SMH_SENS_TYPE_CURRENT,0,0,GPIO_PIN_15,'A',ADC_CHANNEL_0);
	record->Locked = true;
	cur_el = SMH_SensorDB_Add(cur_el,record);


	record = Sensor_Init(TEMP1,"TEMP1",SENSOR_ANALOG,SENSOR_IN,SENSOR_INT,SENSOR_EVT_OFF,SMH_SENS_TYPE_TEMP,0,5,0,'-',ADC_CHANNEL_TEMPSENSOR);
	record->Locked = true;
	record->Status = SENSOR_UP;
	record->SendEvents =true;
	cur_el = SMH_SensorDB_Add(cur_el,record);

	record = Sensor_Init(VREF,"VREF",SENSOR_ANALOG,SENSOR_IN,SENSOR_INT,SENSOR_EVT_OFF,SMH_SENS_TYPE_VOLTAGE,0,0,0,'-',ADC_CHANNEL_VREFINT);
	record->Locked = true;
	record->Status = SENSOR_UP;
	record->SendEvents = true;
	cur_el = SMH_SensorDB_Add(cur_el,record);




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
//void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	SMH_SensorDescrTypeDef* sensor = GetSensorByPinNum(GPIO_Pin);

	if ((sensor != NULL) && ( ! sensor->isAnalog) && (sensor->Direction == SENSOR_IN )) {

		if ((sensor->SwTime - getUpTime()) > 200 ) {
			SMH_SensValueReply_t* polled = SMH_SensorGetValue(sensor);
			SMHome_SendSensorValue(sensor->id, polled);
			sensor->SwCurState = polled->value;
			sensor->SwTime = getUpTime();
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
 *
 * Generate initial sensors record
 *
 * @param name  	On board connector name  for this port
 * @param SigType 	Signal type digital or analog
 * @param InOut     Signal direction input output
 * @param Location 	internal sensor or connected
 * @param Evt     	Send events on status change
 * @param SensType  Sensor type
 * @param multi   	Power of 10 for sensor value
 * @param time   	time polls interval
 * @param pin    	on chip pin number
 * @param port   	on chip pin port
 * @param adc_channel  ADC port for conversation
 */
SMH_SensorDescrTypeDef* Sensor_Init(SensorID_t id,
		char* name,
		bool SigType,
		bool InOut,
		bool Location,
		bool Evt,
		SensorType_t SensType,
		uint8_t multi,
		uint16_t time, // pollin period
		uint16_t pin,
		char port,
		uint32_t adc_channel
)
{

	SMH_SensorDescrTypeDef* Sensor = (SMH_SensorDescrTypeDef*) malloc(sizeof(SMH_SensorDescrTypeDef));
	if (Sensor != NULL) {
		Sensor->id = id;
		for ( int i=0; i < SENSOR_NAME_LEN ; i++ ) {
			if ( i >= strlen(name)) {
				Sensor->Name[i] = ' ';
			}
			else {
				Sensor->Name[i] = name[i];
			}
		}
		Sensor->Status = SENSOR_DOWN;
		Sensor->Direction = InOut;
		Sensor->Locked = false;
		Sensor->SType = SensType;
		Sensor->Location = Location;
		Sensor->SendEvents = Evt;
		Sensor->isAnalog = SigType;
		Sensor->ValMultiplyer = multi;
		Sensor->CheckTime = time;
		Sensor->PinNum = pin;
		Sensor->PinPort = port;
		Sensor->TimeCount = 0;
		Sensor->adc_ch = adc_channel;
		Sensor->adc_rank = 0;
	}
	else {
		SMHome_error(RC_NO_MEM);
	}

	return 	Sensor;
}

/**
 * Switch sensor OFF
 *
 * @param sensor
 * @return
 */

uint8_t Sensor_OFF(SMH_SensorDescrTypeDef* sensor) {

	//	sensor->SendEvents = SENSOR_EVT_OFF;
	sensor->ValMultiplyer = 0;
	sensor->CheckTime = 0;

	if ( sensor->PinPort != '-') {
		HAL_GPIO_DeInit(GetPortNum(sensor->PinPort), sensor->PinNum);
	}

	// TODO undef GPIO interrupt

#ifdef DEBUG_PRINT_UART
	if ( sensor->id == UTX ) {
		HAL_UART_MspInit(&huart2);
	}
#endif
	sensor->Status = SENSOR_DOWN;
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

	if ( sensor->PinPort == '-') {
		sensor->Status = SENSOR_UP;
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
	HAL_GPIO_DeInit(GetPortNum(sensor->PinPort),sensor->PinPort);
	GPIO_InitStruct.Pin = sensor->PinNum;
	if ( sensor->PinNum == GPIO_PIN_1 ) {
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	}
	else if (sensor->PinNum == GPIO_PIN_2 ) {
		HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	}


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
		if (sensor->Direction == SENSOR_IN) {
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  // GPIO_MODE_IT_RISING_FALLING | GPIO_MODE_IT_RISING
			GPIO_InitStruct.Pull = GPIO_NOPULL;                  //

			if ( sensor->PinNum == GPIO_PIN_1 ) {
				HAL_NVIC_EnableIRQ(EXTI1_IRQn);
				HAL_NVIC_SetPriority(EXTI1_IRQn, 0x1, 0);
			} else if (sensor->PinNum == GPIO_PIN_2 ) {
				HAL_NVIC_EnableIRQ(EXTI2_IRQn);
				HAL_NVIC_SetPriority(EXTI2_IRQn, 0x1, 0);
			} else if ((sensor->PinNum >= GPIO_PIN_10 ) && (sensor->PinNum <=  GPIO_PIN_15)) {
				HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
				HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x1, 0);
			}

		}
		/**   Digital Output */
		else {
			HAL_GPIO_WritePin(GetPortNum(sensor->PinPort), sensor->PinNum, GPIO_PIN_RESET);
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		}
	}

	HAL_GPIO_Init(GetPortNum(sensor->PinPort), &GPIO_InitStruct);
	sensor->Status = SENSOR_UP;
	return IS_OK;
}


/**
 *    Set polling interval for analog in or sending event on digital switch status
 *    for digital sensor polling interval always in.
 *
 * @param sensor
 * @param time_interval
 * @param sendEvent. Send or not event to network
 * @return
 *
 */
uint8_t Sensor_SetPollingOn(SMH_SensorDescrTypeDef* sensor, uint16_t time_interval, bool sendEvent) {

	if (sensor->Direction == SENSOR_IN) {
		sensor->SendEvents = sendEvent;

		if (sensor->isAnalog) {
			sensor->CheckTime = time_interval;
			SMH_ADC_RunConversation();
		}
		else {
			sensor->CheckTime = 0;
			//   Switch on interrupt generation


		}

		return IS_OK;
	}
	else {
		return RC_SENSOR_WRONG_TYPE;
	}
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
		if ( cur->el->PinNum == pin ) {
			if (cur->el->PinPort == 'B' ) {
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
		if ((sensor->isAnalog) &&
				(sensor->Direction == SENSOR_IN) &&
				(sensor->adc_ch != ADC_CHANNEL_0) &&
				(sensor->Status = SENSOR_UP)
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
		if ((sensor->isAnalog) &&
				(sensor->Direction == SENSOR_IN) &&
				(sensor->adc_ch != ADC_CHANNEL_0) &&
				(sensor->Status = SENSOR_UP)
		){

			sConfig.Channel = sensor->adc_ch;
			sConfig.Rank = cur_adc_rank;
			sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;

			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK) {
				sensor->adc_rank = cur_adc_rank;
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
 *  Loop through all pulling sensors, get value end send over CAN
 *  Called from main loop.
 *
 */
void SMH_SensorDOPolling() {

	SMH_SensorListElTypeDef* sensor_ptr = SENSOR_DB;
	SMH_SensorDescrTypeDef* sensor;
	SMH_SensorDescrTypeDef* vref_sensor = GetSensorByID(VREF);

	while  (sensor_ptr != 0) {
		sensor = sensor_ptr->el;
		if ((sensor->Direction == SENSOR_IN) &&
				sensor->SendEvents &&
				(sensor->CheckTime > 0) ) {

			sensor->TimeCount++;
			if (sensor->TimeCount > sensor->CheckTime ) {

				/**
				 *   Time to send polled value for this sensor.
				 */
				SMH_SensValueReply_t polled = {0,0,0};
				if (vref_sensor->adc_rank > 0)
					polled.vref = SMH_ConvResultArray[vref_sensor->adc_rank-1];

				if (sensor->adc_rank > 0) {
					polled.value = SMH_ConvResultArray[sensor->adc_rank-1];
					SMH_ConvResultArray[sensor->adc_rank-1] = 0;
				}

				SMHome_SendSensorValue(sensor->id, &polled);
				sensor->TimeCount=0;
			}
		}
		sensor_ptr = sensor_ptr->next;
	}
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

	if ((sensor->Direction == SENSOR_IN )&& (sensor->Status = SENSOR_UP)) {
		if (( sensor->isAnalog ) && (sensor->adc_ch != ADC_CHANNEL_0 )) {

			if (vref_sensor->adc_rank > 0)
				cur_sens_val.vref = SMH_ConvResultArray[vref_sensor->adc_rank-1];
			if (sensor->adc_rank > 0)
				cur_sens_val.value = SMH_ConvResultArray[sensor->adc_rank-1];

			//			SMHome_SendSensorValue(sensor->id, &polled);


#ifdef TMPSENSOR_CALC_INTERNAL

			if (sensor->id == TEMP1 ) {
				cur_sens_val.value = (uint16_t) TMPSENSOR_getTemperature(cur_sens_val.value,cur_sens_val.vref) * 100;
				cur_sens_val.power_of_ten = -2;
			}
#endif
		}
		else if(( ! sensor->isAnalog ) ) {
			cur_sens_val.value = HAL_GPIO_ReadPin(GetPortNum(sensor->PinPort), sensor->PinNum);
		}
		else if (sensor->adc_ch == ADC_CHANNEL_0) {
			SMHome_SendError(NULL, NULL, RC_ADC_CH_UNUSED);
		}
		else if (sensor->Status == SENSOR_DOWN) {
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

	if ((! sensor->isAnalog) && (sensor->Direction == SENSOR_OUT )) {
		if ( value == 1 ) {
			HAL_GPIO_WritePin(GetPortNum(sensor->PinPort), sensor->PinNum, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(GetPortNum(sensor->PinPort), sensor->PinNum, GPIO_PIN_RESET);
		}
		//		rc = HAL_GPIO_TogglePin(GetPortNum(sensor->PinPort), sensor->PinNum);
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

