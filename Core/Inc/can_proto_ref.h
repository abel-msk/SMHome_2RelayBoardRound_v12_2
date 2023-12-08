/*
 * sh_can_proto.h
 *
 *  Created on: Jul 6, 2023
 *      Author: abel
 */

/**
 *  CAN extended identifier usage:
 *
 *
 *  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 *  |28|27|26|25|24|23|22|21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
 *  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 *  |  |    ADDR               |    PORT   |     SRCADDR           | SRCPORT   |   CMD     |
 *  +--+-----------------------+-----------+-----------------------+-----------+-----------+
 *
 *   REPLY - 1 bit
 *   ADDR  - 8bit  Destination address
 *   PORT  - 4bit  Destination port aka sensor id
 *
 *   SRCADDR - 8bit  Sender address
 *   SRCPORT - 4bit  Sender port, sensor id
 *
 *   CMD  - 4bit Command / Action
 *
 *   CMD 0x00 -  KeepALive,  Reply with Status.  Databits:  uint32_t Uptime in secs
 *
 *
 *
 *  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 *  |28|27|26|25|24|23|22|21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
 *  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 *  |    ADDR               |    PORT   |     SRCADDR           | SRCPORT   |   CMD        |
 *  +-----------------------+-----------+-----------------------+-----------+--------------+
 *
 *  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 *  |28|27|26|25|24|23|22|21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
 *  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 *  |PR|  ADDR              |    PORT   |RL|  SRCADDR           | SRCPORT   |   CMD        |
 *  +--+--------------------+-----------+--+--------------------+-----------+--------------+
 *
 *
 *
 */



/**
 *
 *
 *  Sensors IDs
 *  ============
 *  BOARD - id 0
 *  SW1 - id 1
 *  SW2 - id 2
 *  SN1	- id 3
 *  SN2 - id 4
 *  UTX - id 5
 *  RL1 - id 6
 *  RL2 - id 7
 *  TEMP - id 8

	CMD
	====

	CAN_CMD_GET_STATE = 0, System up time for BOARD,  Send current configuration  for sensor
	CAN_CMD_GET_UID = 1,   Board UID
	CAN_CMD_RESET = 2,     Board Reset

	CAN_CMD_GET_CONFIG = 3,  Send current configuration for sensor by its id
	CAN_CMD_SET_CONFIG = 4,  Configure sensors  by its id

	CAN_CMD_LINK_SENS = 5,
	CAN_CMD_SAVE_SENS = 6,

	CAN_CMD_SEND_SENS_VALUE = 7,    Reply port value
	CAN_CMD_SET_SENS_VALUE = 8,     Set port value

	CAN_CMD_ERROR = 9


	Get board Status/keepalive (CAN_CMD_GET_STATE)
	===============================================

	cmd: 0x00

	Reply
	Byte0                                       Byte1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|                                   	UPTIME
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte2                                       Byte3
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	                               	   	   UPTIME
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+


	Get board UID Request
	=====================
	cmd: 0x01


	Get board UID Reply
	-------------------
	cmd: 0x01


	Reply 1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|         Tot Packets                   |   |      SeqID                                                                        |     Byte 4 - 7   (uint32)
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Reply 2
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|                SeqID + 1                                                          |   |    Byte 2 - 6   (uint32)              |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	...

	Reply n+1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	| 1  |           SeqID + n                                                          |   |     Byte 2 - 6   (uint32)             |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+


	Board Reset Request
	===================

	cmd: 0x02

	Byte1
	+----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+
	|                                       |
	+----+----+----+----+----+----+----+----+



	Configure sensor values (CAN_CMD_SET_CONFIG)
	======================================== =====

	cmd: 0x03  Request


	Sensor ID id;   // The sensor ID map incoming CAN port

	stat  - Status;            //   ON = 1  / OFF = 0
	SignalType;        //   ANALOG = 1 / DIGITAL = 0
	Direction;         //   Signal direction in = 1 / out = 0
	Poll               //   Do Polling
	E_L                //   Send event on switch from high to low value
	E_H                //   Send event on switch from low to high value

	SType;             //   Sensor type. Like Voltage, Temperature etc
	PollingTime;       //   Sensor poll interval in sec



	Byte0                                       Byte1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|         |Stat|SigT| Dir|Poll| E_L|T_H |   |  Poll interval (sec)                  |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte2                                       Byte3
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|         Poll interval (sec)               |             TH_Low                    |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte4                                       Byte5
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	                       TH_Low                        TH_High
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte6                                       Byte7
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
               TH_High
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+




	Configure sensor reply (CAN_CMD_SET_CONFIG)
	-------------------------------------------

	cmd: 0x03  Reply

		Byte1
	+----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+
	|           ОК/ERROR_NUM                |
	+----+----+----+----+----+----+----+----+






	Configure Sensor threshold values (CAN_CMD_LINK_SENS)
	=====================================================


	Dir Flag:   1  - From High to LOw
	Dir Flag:   0  - From LOw to High

	ST Flag:    1  - ON
	Dir Flag:   0  - OFF


	Byte0                                       Byte1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
    |    ON Event ToLow  Dev id                 |   ON Event TO_LOW set value 1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+


	Byte2                                       Byte3
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
    ON Event ToLow set value 1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte4                                       Byte5
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	            ID OUT                                     Value to SET
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte6                                       Byte7
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
			Value to SET
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
















	Configure board request
	=======================

	cmd: 0x03 ; dest->port: 0 Request;


	Byte2                                       Byte3
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|         Master addr                  |   |        Secondary мaster  addr          |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+




	Sensor Get Status Request
	=========================

	cmd: 0x04  Request; Len 0

	Byte1
	+----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+
	|
	+----+----+----+----+----+----+----+----+


	Sensor Get Status Reply
	-----------------------

	cmd: 0x04  Reply


	Byte1                                       Byte2
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
    |           Value type                  |   |           Power of 10 Signed int      |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+


	Byte3                                       Byte4
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
                                    Current sensor value
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte5
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	                               Current sensor value
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte7                                       Byte8
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
                                  Current sensor value
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+




	Sensor send polled value (CAN_CMD_CUR_SENS_VALUE)
	=================================================

	Reply
	-----
	cmd: 0x07


	Byte0                                       Byte1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
    |   Event Type                          |   |     Power of 10     signed int        |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+


	Byte2                                       Byte3
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
                                    Current sensor value
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte4                                       Byte5
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	                               Current vref value
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	Byte6                                       Byte7
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+

	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+






	Sensor Switch (CAN_CMD_SET_SENS_VALUE)
	=======================================

	Request
	-------
	cmd: 0x08  Request; Len 1

	Byte0
	+----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+
	|                New Value              |
	+----+----+----+----+----+----+----+----+

	Value = 1 -  ON
    Value = 0 -  OFF


	Reply
	-----
	cmd: 0x08  Reply

	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
    |                                   ERROR NUM                                       |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+



	Send Error (CAN_CMD_ERROR)
	==========================

	Byte0                                       Byte1
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
	|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|   |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+
    |                                   ERROR NUM                                       |
	+----+----+----+----+----+----+----+----+   +----+----+----+----+----+----+----+----+







 */



#ifndef INC_CAN_PROTO_REF_H_
#define INC_CAN_PROTO_REF_H_


#include <stdint.h>


#define CANID_ADDR_BCAST 0xFF

#define CANID_PORT_SENSOR 0x1     //  Send from sensor
#define CANID_SPORT_SENSOR 0x01   //  ID for Temperature sensor

typedef enum  {
	CAN_CMD_GET_STATE = 0,
	CAN_CMD_GET_UID = 1,
	CAN_CMD_RESET = 2,

	CAN_CMD_GET_CONFIG = 3,
	CAN_CMD_SET_CONFIG = 4,

	CAN_CMD_LINK_SENS = 5,
	CAN_CMD_SAVE_SENS = 6,

	CAN_CMD_SEND_SENS_VALUE = 7,
	CAN_CMD_SET_SENS_VALUE = 8,

	CAN_CMD_ERROR = 9
} CAN_cmd;


typedef enum {
	EVT_POOLING_VALUE,
	EVT_REPLY_VALUE,
	EVT_ERROR
} CAN_evt_type;

//#define CAN_CMD_NONE 0x01         //  Empty packet
//#define CAN_CMD_REBOOT 0x02       //  Reboot
//#define CAN_CMD_GETSTATUS 0x03    //  Request for sensor (by port) status
//#define CAN_CMD_ON 0x03           //  Request for sensor (by port) off or stop send vales
//#define CAN_CMD_OFF 0x04          //  Request for sensor (by port) ON or start send vales
//#define CAN_CMD_TOGGLE 0x05       //  Request for sensor (by port) toggle status
//#define CAN_CMD_POL 0x06          //  Request for set polling interval
//#define CAN_CMD_ERROR 0x07        //  Send Error


//#define CANID_SET_REPLY(x) ((x & 0x01) << 28)
//#define CANID_SET_ADDR(x) (x << 20)
//#define CANID_SET_PORT(x) ((x & 0x0f) << 16)
//#define CANID_SET_ADDR_S(x) (x << 8)
//#define CANID_SET_PORT_S(x) ((x & 0x0f) << 4)
//#define CANID_SET_CMD(x)  (x & 0x0f)
//
//#define CANID_IS_REPLY(x) ((x >> 28) & 0x01)
//#define CANID_GET_ADDR(x) ((x >> 20) & 0xff)
//#define CANID_GET_PORT(x) ((x >> 16) & 0x0f)
//#define CANID_GET_ADDR_S(x) ((x >> 8) & 0xff)
//#define CANID_GET_PORT_S(x) ((x >> 4) & 0x0f)
//#define CANID_GET_CMD(x)  (x & 0x0f)


//#define CANID_SET_REPLY(x) ((x & 0x01) << 28)
#define CANID_SET_ADDR(x) (x << 21)
#define CANID_SET_PORT(x) ((x & 0x0f) << 17)
#define CANID_SET_ADDR_S(x) (x << 9)
#define CANID_SET_PORT_S(x) ((x & 0x0f) << 5)
#define CANID_SET_CMD(x)  (x & 0x1f)

//#define CANID_IS_REPLY(x) ((x >> 28) & 0x01)
#define CANID_GET_ADDR(x) ((x >> 21) & 0xff)
#define CANID_GET_PORT(x) ((x >> 17) & 0x0f)
#define CANID_GET_ADDR_S(x) ((x >> 9) & 0xff)
#define CANID_GET_PORT_S(x) ((x >> 5) & 0x0f)
#define CANID_GET_CMD(x)  (x & 0x1f)



typedef struct {
	uint8_t addr;
	uint8_t port;
}
CanAddr;

typedef struct {
	CanAddr dest;
	CanAddr src;
	CAN_cmd cmd;
	uint8_t data[8];
	uint8_t len;
} CanPacket;





#endif /* INC_CAN_PROTO_REF_H_ */
