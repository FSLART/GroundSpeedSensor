/*
 * can_bus.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Low level CAN1 (CAN_RX / CAN_TX, transceiver standby on CAN_STBY).
 *  RX: FIFO0 interrupt -> software queue, read with CAN_Bus_Receive().
 *  TX: software queue -> 3 HW mailboxes, pushed by CAN_Bus_Service().
 *  The protocol itself lives in can_interface.c.
 */

#ifndef INC_CAN_BUS_H_
#define INC_CAN_BUS_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint32_t id;
	uint32_t ide;       // CAN_ID_STD / CAN_ID_EXT
	uint8_t dlc;
	uint8_t data[8];
} CanFrame_t;

HAL_StatusTypeDef CAN_Bus_Init(uint32_t rxStdId);  // HW filter accepts only this standard ID
void CAN_Bus_Service(void);

HAL_StatusTypeDef CAN_Bus_Send(uint32_t stdId, uint8_t dlc, const uint8_t *data);
bool CAN_Bus_Receive(CanFrame_t *frame);

uint16_t CAN_Bus_TxQueueUsed(void);
bool CAN_Bus_IsStarted(void);

/* diagnostics */
uint8_t CAN_Bus_GetTEC(void);
uint8_t CAN_Bus_GetREC(void);
bool CAN_Bus_IsBusOff(void);
uint32_t CAN_Bus_GetTxDropped(void);
uint32_t CAN_Bus_GetRxDropped(void);
uint32_t CAN_Bus_GetTxAborted(void);

#endif /* INC_CAN_BUS_H_ */
