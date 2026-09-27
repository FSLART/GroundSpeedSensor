/*
 * can_bus.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 */

#include "can_bus.h"
#include "app_config.h"
#include "can.h"
#include "timebase.h"

#define TX_MASK (CAN_TX_QUEUE_LEN - 1u)
#define RX_MASK (CAN_RX_QUEUE_LEN - 1u)

#if ((CAN_TX_QUEUE_LEN & TX_MASK) != 0) || ((CAN_RX_QUEUE_LEN & RX_MASK) != 0)
#error "CAN queue lengths must be powers of 2"
#endif

#define ALL_TX_MAILBOXES (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)
#define CAN_RESTART_PERIOD_MS   1000u   // retry HAL_CAN_Start if it failed (transceiver not ready at boot)

/* TX queue: only touched from the main loop */
static CanFrame_t txQueue[CAN_TX_QUEUE_LEN];
static uint16_t txHead = 0;
static uint16_t txTail = 0;

/* RX queue: head written by the ISR, tail by the main loop */
static CanFrame_t rxQueue[CAN_RX_QUEUE_LEN];
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;

static bool started = false;
static uint32_t rxFilterId = 0;
static uint32_t lastStartTry = 0;
static uint32_t txDropped = 0;
static volatile uint32_t rxDropped = 0;
static uint32_t txAborted = 0;

static uint32_t lastTxProgress = 0;
static uint32_t freeAfterLastAdd = 3;

HAL_StatusTypeDef CAN_Bus_Init(uint32_t rxStdId) {

	CAN_FilterTypeDef filter = { 0 };

	rxFilterId = rxStdId;
	lastStartTry = getRuntimeMs();

	// transceiver out of standby
	HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);

	/* 32 bit ID/mask: STID[10:0] | EXID[17:0] | IDE | RTR | 0
	 * exact standard ID, data frames only */
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = (uint32_t) ((rxStdId & 0x7FFu) << 5);
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = (uint32_t) (0x7FFu << 5);
	filter.FilterMaskIdLow = 0x0006;    // IDE and RTR must be 0
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
		return HAL_ERROR;
	}

	// without this HAL_CAN_RxFifo0MsgPendingCallback never fires
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		started = false;
		return HAL_ERROR;
	}

	started = true;
	lastTxProgress = getRuntimeMs();

	return HAL_OK;
}

/* ================== TX ================== */

HAL_StatusTypeDef CAN_Bus_Send(uint32_t stdId, uint8_t dlc, const uint8_t *data) {

	if (dlc > 8) {
		dlc = 8;
	}

	uint16_t next = (txHead + 1u) & TX_MASK;

	// full: drop the OLDEST frame, fresh data matters more than stale data
	if (next == txTail) {
		txTail = (txTail + 1u) & TX_MASK;
		txDropped++;
	}

	CanFrame_t *f = &txQueue[txHead];
	f->id = stdId;
	f->ide = CAN_ID_STD;
	f->dlc = dlc;

	for (uint8_t i = 0; i < 8; i++) {
		f->data[i] = (i < dlc) ? data[i] : 0;
	}

	txHead = next;

	return HAL_OK;
}

uint16_t CAN_Bus_TxQueueUsed(void) {
	return (txHead - txTail) & TX_MASK;
}

void CAN_Bus_Service(void) {

	uint32_t now = getRuntimeMs();

	if (!started) {

		// a failed HAL_CAN_Start leaves the handle in ERROR: full re-init
		if ((now - lastStartTry) >= CAN_RESTART_PERIOD_MS) {
			HAL_CAN_DeInit(&hcan1);
			MX_CAN1_Init();
			CAN_Bus_Init(rxFilterId);
		}

		return;
	}
	uint32_t freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

	// a mailbox finished since the last call (or all idle) = bus is moving
	if ((freeLevel == 3u) || (freeLevel > freeAfterLastAdd)) {
		lastTxProgress = now;
	}

	// nobody ACKs (alone on the bus / cable off): don't keep stale frames forever
	if ((freeLevel < 3u) && ((now - lastTxProgress) > CAN_TX_STUCK_TIMEOUT_MS)) {
		HAL_CAN_AbortTxRequest(&hcan1, ALL_TX_MAILBOXES);
		txAborted++;
		lastTxProgress = now;
	}

	while ((txHead != txTail) && (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0u)) {

		CanFrame_t *f = &txQueue[txTail];
		CAN_TxHeaderTypeDef header = { 0 };
		uint32_t mailbox = 0;

		header.StdId = f->id;
		header.IDE = CAN_ID_STD;
		header.RTR = CAN_RTR_DATA;
		header.DLC = f->dlc;
		header.TransmitGlobalTime = DISABLE;

		if (HAL_CAN_AddTxMessage(&hcan1, &header, f->data, &mailbox) != HAL_OK) {
			break;
		}

		txTail = (txTail + 1u) & TX_MASK;
	}

	freeAfterLastAdd = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

/* ================== RX ================== */

bool CAN_Bus_Receive(CanFrame_t *frame) {

	if (rxTail == rxHead) {
		return false;
	}

	*frame = rxQueue[rxTail];
	rxTail = (rxTail + 1u) & RX_MASK;

	return true;
}

// HAL calls this when a message lands on FIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	if (hcan->Instance != CAN1) {
		return;
	}

	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u) {

		CAN_RxHeaderTypeDef header;
		uint8_t data[8];

		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
			return;
		}

		uint16_t next = (rxHead + 1u) & RX_MASK;

		if (next == rxTail) {
			rxDropped++;
			continue;
		}

		CanFrame_t *f = &rxQueue[rxHead];
		f->ide = header.IDE;
		f->id = (header.IDE == CAN_ID_STD) ? header.StdId : header.ExtId;
		f->dlc = (header.DLC > 8u) ? 8u : (uint8_t) header.DLC;

		for (uint8_t i = 0; i < 8; i++) {
			f->data[i] = data[i];
		}

		rxHead = next;
	}
}

/* ================== DIAGNOSTICS ================== */

bool CAN_Bus_IsStarted(void) {
	return started;
}

uint8_t CAN_Bus_GetTEC(void) {
	return (uint8_t) ((hcan1.Instance->ESR & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
}

uint8_t CAN_Bus_GetREC(void) {
	return (uint8_t) ((hcan1.Instance->ESR & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
}

bool CAN_Bus_IsBusOff(void) {
	return (hcan1.Instance->ESR & CAN_ESR_BOFF) != 0u;
}

uint32_t CAN_Bus_GetTxDropped(void) {
	return txDropped;
}

uint32_t CAN_Bus_GetRxDropped(void) {
	return rxDropped;
}

uint32_t CAN_Bus_GetTxAborted(void) {
	return txAborted;
}
