/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

uint32_t TxMailbox;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 *
 * Prepare filter for incoming CAN packets.  Set two filter bank
 * first for this board ID and second for broadcast.
 *
 * @param hcan
 */
void  CAN_ListenFilter_Init(CAN_HandleTypeDef *hcan, uint8_t myAddr) {

	CAN_FilterTypeDef sFilterConfig;

	uint32_t FilterMask = 0x00 | CANID_SET_ADDR(0xFF);
	uint32_t FilterID   = 0x00 | CANID_SET_ADDR(myAddr);

//
//	printf("Configure FilterID: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\n",
//			BYTE_TO_BINARY(FilterID>>24), BYTE_TO_BINARY(FilterID>>16),
//			BYTE_TO_BINARY(FilterID>>8), BYTE_TO_BINARY(FilterID));
//
//	printf("Configure FilterMask: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\n",
//			BYTE_TO_BINARY(FilterMask>>24), BYTE_TO_BINARY(FilterMask>>16),
//			BYTE_TO_BINARY(FilterMask>>8), BYTE_TO_BINARY(FilterMask));


	//  Configure Filter for Ext ID in bank 1
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

	sFilterConfig.FilterIdHigh = (uint16_t)(FilterID >> 13);            // старшая часть первого "регистра фильтра"
	sFilterConfig.FilterIdLow = (uint16_t)(FilterID << 3) | 0x04;       // младшая часть первого "регистра фильтра"
	sFilterConfig.FilterMaskIdHigh = (uint16_t)(FilterMask >> 13);      // старшая часть второго "регистра фильтра"
	sFilterConfig.FilterMaskIdLow = (uint16_t)(FilterMask << 3) | 0x04; // младшая часть второго "регистра фильтра"

	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
	    Error_Handler();
	}


	//  Configure Filter for get Broadcast in bank 2

	FilterID   = 0x00 | CANID_SET_ADDR(0xFF);

	sFilterConfig.FilterBank = 1;                                       // which filter bank to use from the assigned ones
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

	sFilterConfig.FilterIdHigh = (uint16_t)(FilterID >> 13);            // старшая часть первого "регистра фильтра"
	sFilterConfig.FilterIdLow = (uint16_t)(FilterID << 3) | 0x04;       // младшая часть первого "регистра фильтра"
	sFilterConfig.FilterMaskIdHigh = (uint16_t)(FilterMask >> 13);      // старшая часть второго "регистра фильтра"
	sFilterConfig.FilterMaskIdLow = (uint16_t)(FilterMask << 3) | 0x04; // младшая часть второго "регистра фильтра"

	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
	    Error_Handler();
	}



}

/**
 *
 * Handle error during CAN send/receive process.  Print error code to DEBUG/UART output.
 * @param hcan
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
#ifdef DEBUG_PRINT_UART
			uint32_t er = HAL_CAN_GetError(hcan);
			println("ER CAN %lu %08lX", er, er);
#endif
	}
}



/**
 *
 * Handle interrupt when can packet arrived.  Load data, and place to queue.
 * @param hcan
 *
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	CAN_RxHeaderTypeDef RxHeader;
	CanPacket pkt;
	uint8_t RxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
#ifdef DEBUG_PRINT_UART
		println("Cannot receive CAN message.");
#endif
		Error_Handler();
	}
	else {

		pkt.dest.addr = CANID_GET_ADDR(RxHeader.ExtId);
		pkt.dest.port = CANID_GET_PORT(RxHeader.ExtId);
		pkt.src.addr = CANID_GET_ADDR_S(RxHeader.ExtId);
		pkt.src.port = CANID_GET_PORT_S(RxHeader.ExtId);
		pkt.cmd = CANID_GET_CMD(RxHeader.ExtId);
		pkt.len = RxHeader.DLC;

		for (int i=0; i < pkt.len; i++) {
			pkt.data[i] = RxData[i];
		}

		//  Put new packet in queue
		q_Push(&pkt);

	}
}


/**
 *
 * Handle CAN packet send.
 * @param TxHeader
 * @param TxData
 * @return
 */
uint8_t CAN_Send_Packet(CAN_TxHeaderTypeDef *TxHeader, uint8_t *TxData ) {

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

    if(HAL_CAN_AddTxMessage(&hcan, TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
#ifdef DEBUG_PRINT_UART

            println("ER SEND %u %08lX", 8, hcan.ErrorCode);
#endif
            return hcan.ErrorCode;
    }
    else return IS_OK;

}



/* USER CODE END 1 */
