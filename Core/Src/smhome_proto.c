/*
 * selector.c
 *
 *  Created on: Jul 16, 2023
 *      Author: abel
 */

#include "smhome_proto.h"
#include "stm32f1xx_hal.h"
#include "rtc.h"

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {0,};
CanPacket* pkt = NULL;

/**
 *
 * Main entry pint for select action by received CAN packet.
 * Processed one packet (first in queue). After processed^ packet deleted from queue.
 *
 * @param q_get
 */
void SMHome_InputSelector(CanPacket* pkt) {
	uint8_t rc = 0;
	CanAddr from_me;
	from_me.port = pkt->dest.port; // We receive request for this port (sensor)
	from_me.addr = THIS_CANID;
	CanAddr* reply_to_addr = &(pkt->src);
	CanAddr* from = &from_me;

	switch (pkt->cmd) {
	case CAN_CMD_GET_UID:
		rc = SMHome_SendUID(reply_to_addr, from);
		break;

	case CAN_CMD_GET_CONFIG:
	case CAN_CMD_GET_STATE:
		if ( pkt->dest.port == (SensorID_t)BOARD ) {
			rc = SMHome_SendUpTime(reply_to_addr, from);
		}
		else
			rc = SMHome_SendSensorConf(reply_to_addr,from);

	case CAN_CMD_SET_CONFIG:
		rc = SMHome_NetConf(reply_to_addr, pkt->dest.port, pkt->data, pkt->len);
		break;

	case CAN_CMD_SEND_SENS_VALUE:
		rc = SMHome_ReplySensorValue(reply_to_addr,from);
		break;

	case CAN_CMD_SET_SENS_VALUE:
		rc = SMHome_SwitchSensorState(reply_to_addr,from,pkt->data[0]);
		break;

	case CAN_CMD_SET_SENS_THRESHOLD:
		rc = SMHome_NetConfThreshold(reply_to_addr, pkt->dest.port, pkt->data, pkt->len);
		break;
	default:
		rc = RC_CAN_UNKNOWN_CMD;
	}

	if (rc != IS_OK ){
		SMHome_SendError(reply_to_addr,from,rc);
//		println ("ER CAN_ERROR CMD.");
	}
}

/**
 *
 *   Prepare CAN Header for new packet
 *
 * @param src  - Source address and port
 * @param dest - Destination address and port
 * @param cmd -  Packet type, command
 * @return - prepared header
 *
 */
CAN_TxHeaderTypeDef *CAN_TxHeader_Create(CanAddr* dest, CanAddr* src, CAN_cmd cmd, bool is_reply) {

	uint32_t ReplyID=0;

	ReplyID = ReplyID | CANID_SET_ADDR(dest->addr);
	ReplyID = ReplyID | CANID_SET_PORT(dest->port);
	ReplyID = ReplyID | CANID_SET_ADDR_S(THIS_CANID);
	ReplyID = ReplyID | CANID_SET_PORT_S(src->port);
	ReplyID = ReplyID | CANID_SET_CMD(cmd);

	// Prepare reply packet for send

	TxHeader.StdId = 0;
	TxHeader.ExtId = ReplyID;    // Receiver's address and port
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_EXT;   // Use extended Can ID
	TxHeader.TransmitGlobalTime = 0;

	return &TxHeader;
}

/**
 *
 * Called when CAN request "Get_UID" received.
 * Send this this CPU UUID as replay.
 * Usually UID required 4 packets length to be send.
 *
 * @param src
 * @param dest
 * @param is_reply
 * @return
 */
uint8_t SMHome_SendUID(CanAddr *send_to, CanAddr *from) {

	CAN_TxHeaderTypeDef *TxHeaderPtr;
	//Get Random sequence number

	uint16_t short_randval =(uint16_t)(*idBase2 & *idBase3 & *idBase0 & *idBase0 ) & 0x3FFF;
//	uint16_t short_randval = (uint16_t)(TM_RNG_Get() & 0x3FFF);

	TxHeaderPtr = CAN_TxHeader_Create(send_to, from, CAN_CMD_GET_UID, true);    // switch source and dest on reply packet
//	TxHeaderPtr->ExtId = TxHeaderPtr->ExtId | CANID_SET_REPLY(1);

	//  Prepare send data

	//  PACKET 1
	TxData[0] =  3;
	TxData[1] = (uint8_t) (short_randval >> 8);
	TxData[2] = (uint8_t)  short_randval;

	TxData[3] = (uint8_t) (*idBase0 >> 8);
	TxData[4] = (uint8_t) (*idBase0);
	TxData[5] = (uint8_t) (*idBase1 >> 8);
	TxData[6] = (uint8_t) (*idBase1);

	TxHeaderPtr->DLC = 7;

    if ( CAN_Send_Packet(TxHeaderPtr,TxData) != IS_OK) {
    	return RC_CAN_TRANSMIT_ERR;
    }

    //  PACKET 2
    short_randval++;

	TxData[0] = 1 << 7;  //  Set reply bit
	TxData[0] = TxData[0] | (uint8_t) (short_randval >> 8);
	TxData[1] = (uint8_t)  short_randval;

	TxData[2] = (uint8_t) (*idBase2 >> 24);
	TxData[3] = (uint8_t) (*idBase2 >> 16);
	TxData[4] = (uint8_t) (*idBase2 >> 8);
	TxData[5] = (uint8_t) (*idBase2);
	TxHeaderPtr->DLC = 6;

    if ( CAN_Send_Packet(TxHeaderPtr,TxData) != IS_OK) {
    	return RC_CAN_TRANSMIT_ERR;
    }

    //  PACKET 3
    short_randval++;

	TxData[0] = 1 << 7;  //  Set reply bit
	TxData[0] = TxData[0] | (uint8_t) (short_randval >> 8);
	TxData[1] = (uint8_t)  short_randval;

	TxData[2] = (uint8_t) (*idBase3 >> 24);
	TxData[3] = (uint8_t) (*idBase3 >> 16);
	TxData[4] = (uint8_t) (*idBase3 >> 8);
	TxData[5] = (uint8_t) (*idBase3);
	TxHeaderPtr->DLC = 6;

    if ( CAN_Send_Packet(TxHeaderPtr,TxData) != IS_OK) {
    	return RC_CAN_TRANSMIT_ERR;
    }

    return IS_OK;
}



/**
 *
 * Prepare and send reply packet for GetUptime command.
 *
 * @param src
 * @param dest
 * @param is_reply
 * @return
 */
uint8_t SMHome_SendUpTime(CanAddr *send_to, CanAddr *from) {
	CAN_TxHeaderTypeDef *TxHeaderPtr;

	TxHeaderPtr = CAN_TxHeader_Create(send_to, from, CAN_CMD_GET_STATE, true);

	uint32_t uptime_counter = getUpTime();

	TxData[0] = TxData[0] | (uint8_t) (uptime_counter >> 24);
	TxData[1] = (uint8_t) (uptime_counter >> 16);
	TxData[2] = (uint8_t) (uptime_counter >> 8);
	TxData[3] = (uint8_t) uptime_counter;

	TxHeaderPtr->DLC = 4;

    if ( CAN_Send_Packet(TxHeaderPtr,TxData) != IS_OK) {
    	return RC_CAN_TRANSMIT_ERR;
    }
    return IS_OK;
}

/**
 * Process request for configure sensors type and parameters
 *
 * @param reply_to
 * @param id    sensor id to configure ( will used in reply send as source port num)
 * @param data  received data
 * @param len   received data length
 * @return
 */

uint8_t SMHome_NetConf(CanAddr* reply_to, SensorID_t id, uint8_t data[] , uint8_t len){
	uint8_t rc = IS_OK;
	CanAddr from;
	from.addr = THIS_CANID;
	from.port = id;

	if (id == (SensorID_t)BOARD) {
		CAN_Masters[0] = data[2];
		CAN_Masters[1] = data[3];
	}
	else {

		//  Check if request from master
		SMH_SensorDescrTypeDef *sensor;
		sensor = GetSensorByID(id);

		if (sensor == NULL) {
			rc =  RC_SENSOR_NOT_FOUND;
		}
		else {
			// Get SensorType

			if ( ! sensor->isLocked) {

				if ( IS_SENS_ANALOG(data[0]) )
					sensor->isAnalog = true;
				else
					sensor->isAnalog = false;

				if (IS_SENS_INPUT(data[0]))
					sensor->isInput = true;
				else
					sensor->isInput = false;

			}

			sensor->isPolling = IS_SENS_POLL(data[0]);
			if ( len >= 3) {

				sensor->pollingInterval = data[1] << 8;
				sensor->pollingInterval = sensor->pollingInterval | (data[2] & 0xFF);
			}

			if ( sensor->isInput && sensor->isPolling)
				rc = Sensor_SetPolling(sensor, sensor->pollingInterval);

			sensor->isEventOnLow = IS_SENS_EVT_LOW(data[0]);
			sensor->isEventOnHigh = IS_SENS_EVT_HIGH(data[0]);


			if (rc == IS_OK) {
				if ( IS_SENS_ON(data[0])) {
					rc = Sensor_ON(sensor);
				}
				else {
					rc = Sensor_OFF(sensor);
				}
			}
		}
	}

	if (rc == IS_OK) {
		SMHome_SendSensorConf(reply_to, &from);
	}

	return rc;

}

/**
 *
 *  Add threshold rules for sensor
 *
 * @param reply_to
 * @param id
 * @param data
 * @param len
 * @return
 */
uint8_t SMHome_NetConfThreshold(CanAddr* reply_to, SensorID_t id, uint8_t data[] , uint8_t len) {

	uint8_t rc = IS_OK;
//	CanAddr from;
//	from.addr = THIS_CANID;
//	from.port = id;

	SMH_SensorDescrTypeDef *sensor;
	sensor = GetSensorByID(id);

	bool isON = (data[0] >> 7 == 1);
	bool isToLow = (data[0] >> 6 == 1);
	uint8_t ruleId = (data[0] & 0b00111111);

	if (! isON) {
		Sensor_ClearThreshold(sensor,ruleId);
	}
	else if (len >= 4) {
		uint16_t thValue = 0;
		uint16_t sensorValue = 0;
		uint8_t sensorId = 0;

		if ( len >= 3)
			thValue = data[1] << 8 | data[2];
		if ( len >= 4)
			sensorId = data[3];
		if ( len >= 6)
			sensorValue = data[4] << 8 | data[5];

		//TODO: sensorId - check  sensor ID enum

		rc = Sensor_SetThreshold(sensor, ruleId, isToLow, thValue, sensorId, sensorValue);

	}

	return rc;
}



/**
 *  Send current sensor configuration
 * @param send_to requester or master address
 * @param id sensor id ( will send as source port)
 * @return
 */
uint8_t SMHome_SendSensorConf(CanAddr* send_to, CanAddr* from) {
	CAN_TxHeaderTypeDef *TxHeaderPtr;
	SMH_SensorDescrTypeDef *sensor;
	sensor = GetSensorByID((SensorID_t)from->port);
	TxHeaderPtr = CAN_TxHeader_Create(send_to, from, CAN_CMD_GET_CONFIG, true);

	for (int i=0; i < 8; i++) {  // Clear data bytes
		TxData[i] = 0;
	}

	if ( sensor->isEventOnHigh )
		TxData[0] =  TxData[0] | 0x1;
	if ( sensor->isEventOnLow )
		TxData[0] =  TxData[0] | 0b10;
	if ( sensor->isPolling )
		TxData[0] =  TxData[0] | 0b100;
	if ( sensor->isInput )
		TxData[0] =  TxData[0] | 0b1000;
	if ( sensor->isAnalog )
		TxData[0] =  TxData[0] | 0b10000;
	if ( sensor->status )
		TxData[0] =  TxData[0] | 0b100000;


	// Get Timeout
	TxData[1] = sensor->pollingInterval >> 8;
	TxData[2] = sensor->pollingInterval & 0xFF;

	TxHeaderPtr->DLC = 3;

    if ( CAN_Send_Packet(TxHeaderPtr,TxData) != IS_OK) {
    	return RC_CAN_TRANSMIT_ERR;
    }

    return IS_OK;
}


/**
 *
 * Check from->port state and reply
 * @param send_to
 * @param from
 * @return
 */

uint8_t SMHome_ReplySensorValue(CanAddr* send_to, CanAddr* from) {
	uint8_t rc=IS_OK;
	CAN_TxHeaderTypeDef *TxHeaderPtr;
	SMH_SensorDescrTypeDef *sensor;
	sensor = GetSensorByID((SensorID_t)from->port);

	SMH_SensValueReply_t* current = SMH_SensorGetValue(sensor);

	TxHeaderPtr = CAN_TxHeader_Create(send_to, from, CAN_CMD_SEND_SENS_VALUE, true);

	TxData[0] = EVT_REPLY_VALUE;
	TxData[1] = current->power_of_ten;
	TxData[2] = (uint8_t) (current->value >> 8);
	TxData[3] = (uint8_t) current->value;
	TxData[4] = (uint8_t) (current->vref >> 8);
	TxData[5] = (uint8_t) current->vref;

	TxHeaderPtr->DLC = 6;

    if ( CAN_Send_Packet(TxHeaderPtr,TxData) != IS_OK) {
    	rc=RC_CAN_TRANSMIT_ERR;
    }

//    free(current);
	return rc;
}


/**
 *
 * Send sensor/port value for specified sensor
 * @param sensor_id
 * @param value
 * @return
 */
uint8_t SMHome_SendSensorValue(SensorID_t sensor_id, SMH_SensValueReply_t* polled) {
	uint8_t rc = IS_OK;
	CanAddr send_to, from;
	CAN_TxHeaderTypeDef *TxHeaderPtr;

	send_to.addr = CAN_Masters[0];
	send_to.port = 0xFF;
	from.addr = THIS_CANID;
	from.port= (uint8_t)sensor_id;

	TxHeaderPtr = CAN_TxHeader_Create(&send_to, &from, CAN_CMD_SEND_SENS_VALUE, true);

	TxData[0] = EVT_POOLING_VALUE;
	TxData[1] = polled->power_of_ten;
	TxData[2] = (uint8_t) (polled->value >> 8);
	TxData[3] = (uint8_t) (polled->value );
	TxData[4] = (uint8_t) (polled->vref >> 8);
	TxData[5] = (uint8_t) polled->vref;

	TxHeaderPtr->DLC = 6;

    if ( CAN_Send_Packet(TxHeaderPtr,TxData) != IS_OK) {
    	return RC_CAN_TRANSMIT_ERR;
    }

	return rc;
}


/**
 *
 * Set sensor value.  Switch sensor.
 *
 * @param send_to
 * @param from
 * @return
 */
uint8_t SMHome_SwitchSensorState(CanAddr* send_to, CanAddr* from, uint8_t value) {
	uint8_t rc = IS_OK;
	SMH_SensorDescrTypeDef *sensor;
	sensor = GetSensorByID((SensorID_t)from->port);

	if (sensor->status == SENSOR_ON)
		rc = SMH_SensorSwitch(sensor, value);
	else {
		rc = RC_SENSOR_OFF_ERR;
	}
	return rc;
}


/**
 *   Reply last request status or just send
 * @param send_to
 * @param from
 * @param rc
 * @return
 */
uint8_t SMHome_SendError(CanAddr* send_to, CanAddr* from, uint16_t rc_value) {

	CanAddr local_send_to, local_from;
	CAN_TxHeaderTypeDef *TxHeaderPtr;

	if (send_to == NULL) {
		local_send_to.addr = CAN_Masters[0];
		local_send_to.port = 0xF;
	}
	else {
		local_send_to.addr = send_to->addr;
		local_send_to.port = send_to->port;
	}

	local_from.addr = THIS_CANID;
	if (from == NULL) {
		local_from.port = 0xF;
	}
	else {
		local_from.port=from->port;
	}

	TxHeaderPtr = CAN_TxHeader_Create(&local_send_to, &local_from, CAN_CMD_ERROR, true);
	TxData[0] = (uint8_t) (rc_value >> 8);
	TxData[1] = (uint8_t) rc_value;
	TxHeaderPtr->DLC = 2;

    if ( CAN_Send_Packet(TxHeaderPtr,TxData) != IS_OK) {
//    	return RC_CAN_TRANSMIT_ERR;
    }

	return IS_OK;
}




