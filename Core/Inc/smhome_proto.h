/*
 * selector.h
 *
 *  Created on: Jul 16, 2023
 *      Author: abel
 */

#ifndef INC_SMHOME_PROTO_H_
#define INC_SMHOME_PROTO_H_


#include "main.h"
#include "can.h"
#include "can_proto_ref.h"
#include "can_rx_queue.h"
#include "smhome_sensors.h"

extern RTC_HandleTypeDef hrtc;
extern uint16_t *idBase0;
extern uint16_t *idBase1;
extern uint32_t *idBase2;
extern uint32_t *idBase3;

extern uint8_t CAN_Masters[];

void SMHome_InputSelector(CanPacket*);
uint8_t SMHome_SendUID(CanAddr*, CanAddr*);
uint8_t SMHome_SendUpTime(CanAddr*, CanAddr*);

uint8_t SMHome_NetConf(CanAddr*, SensorID_t, uint8_t[], uint8_t);
SMH_SensorDescrTypeDef* GetSensorByID(SensorID_t);
uint8_t SMHome_SendSensorConf(CanAddr*, CanAddr*);
uint8_t SMHome_SendSensorValue(SensorID_t, SMH_SensValueReply_t*);
uint8_t SMHome_ReplySensorValue(CanAddr*, CanAddr*);
uint8_t SMHome_SwitchSensorState(CanAddr*, CanAddr*, uint8_t);
uint8_t SMHome_SendError(CanAddr*, CanAddr*, uint16_t);
uint8_t SMHome_NetConfThreshold(CanAddr*, SensorID_t, uint8_t*, uint8_t);

#endif /* INC_SMHOME_PROTO_H_ */
