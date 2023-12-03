/*
 * canfifo.h
 *
 *  Created on: Jul 7, 2023
 *      Author: abel
 */

#ifndef INC_CAN_RX_QUEUE_H_
#define INC_CAN_RX_QUEUE_H_
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "main.h"
#include "can_proto_ref.h"


#ifndef CAN_RX_STCK_SIZE
#define CAN_RX_STCK_SIZE 5
#endif

typedef struct QueueEl {
	CanPacket* pkt;
	struct QueueEl* next;
} QueueEl;


typedef struct {
	QueueEl* first;
	QueueEl* last;
	int count;
	int size;
} CAN_PktQueueTypeDef;

void q_Init(int);
CanPacket* q_get(CAN_PktQueueTypeDef*);
QueueEl* q_CreateEl(CanPacket*);
void q_ClearPkt(CanPacket*);
QueueEl* q_Push(CanPacket*);
CanPacket* q_Pop();
int q_Size();
bool q_isFull();
bool q_isEmpty();

//bool q_isEmpty(CAN_PktQueueTypeDef*);
//bool q_isFull(CAN_PktQueueTypeDef*);
//int q_size(CAN_PktQueueTypeDef*);
//void q_insert(CAN_PktQueueTypeDef*, CanPacket*);
//CanPacket* q_remove(CAN_PktQueueTypeDef*);


#endif /* INC_CAN_RX_QUEUE_H_ */
