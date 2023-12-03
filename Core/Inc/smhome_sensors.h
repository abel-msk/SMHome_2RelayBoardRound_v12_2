/*
 * SMHome_Sensors.h
 *
 *  Created on: Jul 16, 2023
 *      Author: abel
 */


/* Internal Sensor DB
 *
 *
 */

#ifndef INC_SMHOME_SENSORS_H_
#define INC_SMHOME_SENSORS_H_

#include "main.h"


#define SENSOR_NAME_LEN 4

#define SENSOR_UP true
#define SENSOR_DOWN false
#define SENSOR_ON true
#define SENSOR_OFF false
#define SENSOR_IN true
#define SENSOR_OUT false
#define SENSOR_DIGITAL false
#define SENSOR_ANALOG true
#define SENSOR_INT false
#define SENSOR_EXT false
#define SENSOR_EVT_ON true
#define SENSOR_EVT_OFF false


#define IS_SENS_IN(x) ((x & 0x01) == 1)
#define IS_SENS_ANALOG(x) ((x & 0b10) == 0b10)
#define IS_SENS_EVT_ON(x) ((x & 0b100) == 0b100)
#define IS_SENS_ON(x) ((x & 0b1000) == 0b1000)



//#define SMH_SENS_TYPE_UNKNOWN 0
//#define SMH_SENS_TYPE_TEMP 1     // temperature
//#define SMH_SENS_TYPE_PRESH 2    // pressure
//#define SMH_SENS_TYPE_HUM 3      // humidity
//#define SMH_SENS_TYPE_LIGHT 4    // illumination
//#define SMH_SENS_TYPE_AIR 5
//#define SMH_SENS_TYPE_TRQUE 6
//#define SMH_SENS_TYPE_SPEED 7
//#define SMH_SENS_TYPE_ANGLE 8
//#define SMH_SENS_TYPE_VOLTAGE 9
//#define SMH_SENS_TYPE_CURRENT 10
//#define SMH_SENS_TYPE_SWITCH 11
//#define SMH_SENS_TYPE_ANALOG 12
//#define SMH_SENS_TYPE_FAN    13
//#define SMH_SENS_TYPE_RELAY  14


typedef enum {
	SMH_SENS_TYPE_UNKNOWN = 0,
	SMH_SENS_TYPE_TEMP,
	SMH_SENS_TYPE_PRESH,
	SMH_SENS_TYPE_HUM ,
	SMH_SENS_TYPE_LIGHT ,
	SMH_SENS_TYPE_AIR ,
	SMH_SENS_TYPE_TRQUE,
	SMH_SENS_TYPE_SPEED,
	SMH_SENS_TYPE_ANGLE,
	SMH_SENS_TYPE_VOLTAGE,
	SMH_SENS_TYPE_CURRENT,
	SMH_SENS_TYPE_SWITCH,
	SMH_SENS_TYPE_ANALOG,
	SMH_SENS_TYPE_FAN,
	SMH_SENS_TYPE_RELAY,
	SMH_SENS_TYPE_MAX,
} SensorType_t;

//SensorType_t SensorType;

// Sensors port enum

typedef enum  {
	BOARD = 0,
	SW1 = 1,
	SW2 = 2,
	SN1 = 3,
	SN2 =4,
	UTX,
	RL1,
	RL2,
	TEMP1,
	ACSENS,
	VREF
} SensorID_t;

//SensorID_t SensorID;


typedef struct {
	SensorID_t id;   // The sensor ID map incoming CAN port
	bool Status;            //   ON / OFF
	bool SendEvents;        //   Send events on status change
	bool isAnalog;        //   DIGITAL-false / ANALOG-true
	bool Direction;         //   Signal direction in / out
	bool Location;          //   Internal or connected
	bool Locked;            //   Can be reconfigured
	char Name [SENSOR_NAME_LEN];          //   Sensor name like SW1 SW1 etc
	SensorType_t SType;     //    Sensor type. Like Voltage, Temperature etc
	uint16_t ValMultiplyer;
	uint16_t CheckTime;     //  Sensor poll interval in ms
	uint16_t  PinNum;
	char  PinPort;
	uint16_t TimeCount;
	uint32_t adc_ch;
	uint32_t SwTime;
	uint8_t  SwCurState;
	uint8_t  adc_rank;
} SMH_SensorDescrTypeDef;

struct SMH_SensorListEl {
	SMH_SensorDescrTypeDef* el;
	struct SMH_SensorListEl *next;

};
typedef  struct SMH_SensorListEl SMH_SensorListElTypeDef;

typedef struct  {
	uint16_t value;
	uint16_t vref;
	int8_t  power_of_ten;
} SMH_SensValueReply_t;


//void HAL_GPIO_EXTI_Callback(uint16_t);

void Sensor_DB_Init();
SMH_SensorDescrTypeDef* Sensor_Init(SensorID_t, char*, bool, bool, bool,bool, uint8_t, uint8_t,uint16_t,uint16_t, char, uint32_t);
SMH_SensorListElTypeDef* SMH_SensorDB_Add(SMH_SensorListElTypeDef*, SMH_SensorDescrTypeDef*);
SMH_SensorDescrTypeDef* SMH_SensorGetByID(uint8_t);
SMH_SensorDescrTypeDef* GetSensorByPinNum(uint16_t);

GPIO_TypeDef* GetPortNum(char);

//SMH_SensorDescrTypeDef* Sensor_Init (SensorID_t,
//		char*,
//		bool,
//		bool,
//		bool,
//		bool,
//		SensorType_t,
//		uint8_t,
//		uint16_t,
//		uint8_t,
//		char
//		uint32_t);



uint8_t Sensor_ON(SMH_SensorDescrTypeDef*);
uint8_t Sensor_OFF(SMH_SensorDescrTypeDef*);
//void Sensor_SetAnalogIn(SMH_SensorDescrTypeDef*);
//void Sensor_SetAnalogOut(SMH_SensorDescrTypeDef*);
//void Sensor_SetDigitIn(SMH_SensorDescrTypeDef*);
//void Sensor_SetDigitOut(SMH_SensorDescrTypeDef*);
uint8_t Sensor_SetPollingOn(SMH_SensorDescrTypeDef*, uint16_t, bool);   // Sensor, time_interval, sendEvent
SMH_SensValueReply_t* SMH_SensorGetValue(SMH_SensorDescrTypeDef*);

void SMH_SensorDOPolling();
void SMH_ADC_RunConversation();
uint8_t SMH_SensorSwitch(SMH_SensorDescrTypeDef*, uint8_t);

#ifdef TMPSENSOR_CALC_INTERNAL
	double TMPSENSOR_getTemperature(uint16_t, uint16_t);
#endif

#endif /* INC_SMHOME_SENSORS_H_ */
