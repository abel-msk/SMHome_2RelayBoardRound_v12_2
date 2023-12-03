/*
 * can_rx_queue.c
 *
 *  Created on: Jul 7, 2023
 *      Author: abel
 */

#include "can_rx_queue.h"
CAN_PktQueueTypeDef qpkt;

/**
 * Initialize queue
 * @param q_size
 */
void q_Init(int q_size) {
	qpkt.size = q_size;
	qpkt.count = 0;
	qpkt.first = NULL;
	qpkt.last = NULL;
}

/**
 *
 * Create new queue element and copy packet contents in it
 * @param pkt
 * @return
 */
QueueEl* q_CreateEl(CanPacket* pkt) {

	QueueEl* el = malloc(sizeof(QueueEl));
	if (el == NULL) { SMHome_error(RC_NO_MEM); }

	el->pkt = malloc(sizeof(CanPacket));
	if (el->pkt == NULL) { SMHome_error(RC_NO_MEM); }

	memcpy(el->pkt, pkt, sizeof(CanPacket));

//	if (pkt->len > 0 ) {
//		for (uint8_t i = 0; i < pkt->len; i++) {
//			el->pkt->data[i] = pkt->data[i];
//		}
//		uint8_t* ar;
//		ar = malloc(sizeof(char)*pkt->len);
//		el->pkt->data = ar;

//		if (el->pkt->data == NULL) { SMHome_error(RC_NO_MEM); }
//		memcpy(el->pkt->data, pkt->data, pkt->len);
//	}
//	else {
//		el->pkt->data  = NULL;
//	}
	return el;
}

/**
 *
 * Free memory of packet saved in queue
 * @param el
 */
void q_ClearPkt(CanPacket* pkt) {
//	if ((pkt->len > 0 ) && (pkt->data != NULL)) {
//		free(pkt->data);
//	}
	free(pkt);
}

/**
 *
 * Insert new packet in queue.
 * @param the packed
 * @return queue element with new packet or NULL in queue is full
 */
QueueEl* q_Push(CanPacket* pkt) {
	QueueEl* el = NULL;

	if (qpkt.count < qpkt.size) {
		el = q_CreateEl(pkt);
		el->next = NULL;

		if ( qpkt.count == 0 ) {
			qpkt.last = el;
			qpkt.first = el;
		}
		else {
			qpkt.last->next = el;
			qpkt.last = el;
		}

		qpkt.count++;
	}
	return el;
}

/**
 *
 *   Remove first saved element in queue and free elements memory.
 *   Packet steel need to be removed by  q_ClearPkt
 * @return saved packet
 */
CanPacket* q_Pop() {
	if ((qpkt.count > 0 ) && (qpkt.first != NULL)) {
		QueueEl* el = qpkt.first;
		qpkt.first = el->next;
		el->next = NULL;
		qpkt.count--;

		CanPacket* ret = el->pkt;
		free(el);
		return ret;
	}
	return NULL;
}




int q_Size() {
	return qpkt.size;
}

bool q_isFull() {
	return qpkt.size == qpkt.count;
}

bool q_isEmpty() {
	if (qpkt.count == 0 ) return true;
	return false;
}
