/*
 * debug_uart.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 */

#include "debug_uart.h"
#include "app_config.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

#define TX_MASK (DEBUG_UART_TX_BUF_SIZE - 1u)

#if (DEBUG_UART_TX_BUF_SIZE & TX_MASK) != 0
#error "DEBUG_UART_TX_BUF_SIZE must be a power of 2"
#endif

static uint8_t txBuf[DEBUG_UART_TX_BUF_SIZE];
static volatile uint32_t txHead = 0;     // written by main loop
static volatile uint32_t txTail = 0;     // written by DMA complete ISR
static volatile uint16_t txDmaLen = 0;
static volatile bool txBusy = false;
static volatile uint32_t droppedBytes = 0;

static uint32_t TxUsed(void) {
	return (txHead - txTail) & TX_MASK;
}

static uint32_t TxFree(void) {
	return DEBUG_UART_TX_BUF_SIZE - 1u - TxUsed();
}

// Starts a DMA transfer of the contiguous block at the tail. Call with IRQs off or from the ISR.
static void TxKick(void) {

	if (txBusy || (txHead == txTail)) {
		return;
	}

	uint32_t tail = txTail;
	uint32_t len = (txHead > tail) ? (txHead - tail) : (DEBUG_UART_TX_BUF_SIZE - tail);

	if (len > 0xFFFFu) {
		len = 0xFFFFu;
	}

	if (HAL_UART_Transmit_DMA(&huart2, &txBuf[tail], (uint16_t) len) == HAL_OK) {
		txDmaLen = (uint16_t) len;
		txBusy = true;
	}
}

void DebugUart_Init(void) {

	txHead = 0;
	txTail = 0;
	txBusy = false;
	droppedBytes = 0;
}

void DebugUart_Write(const uint8_t *data, uint16_t len) {

	uint32_t start = HAL_GetTick();

	while (len > 0) {

		uint32_t space = TxFree();

		if (space == 0) {
			// wait for the DMA to make room, but never hang the loop
			if ((HAL_GetTick() - start) > DEBUG_UART_BLOCK_TIMEOUT_MS) {
				droppedBytes += len;
				return;
			}
			continue;
		}

		uint32_t chunk = (len < space) ? len : space;

		for (uint32_t i = 0; i < chunk; i++) {
			txBuf[txHead] = data[i];
			txHead = (txHead + 1u) & TX_MASK;
		}

		data += chunk;
		len -= (uint16_t) chunk;

		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		TxKick();
		__set_PRIMASK(primask);
	}
}

int DebugUart_Printf(const char *format, ...) {

	char buf[256];
	va_list args;

	va_start(args, format);
	int n = vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	if (n <= 0) {
		return n;
	}

	if (n >= (int) sizeof(buf)) {
		n = sizeof(buf) - 1;
	}

	DebugUart_Write((const uint8_t*) buf, (uint16_t) n);
	return n;
}

char* DebugUart_FormatFloat(char *dst, size_t len, float v, uint8_t decimals) {

	static const int32_t pow10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000 };

	if (decimals > 6) {
		decimals = 6;
	}

	const char *sign = "";
	if (v < 0.0f) {
		sign = "-";
		v = -v;
	}

	int32_t scale = pow10[decimals];
	int32_t ip = (int32_t) v;
	int32_t fp = (int32_t) ((v - (float) ip) * (float) scale + 0.5f);

	if (fp >= scale) {
		ip++;
		fp -= scale;
	}

	if (decimals == 0) {
		snprintf(dst, len, "%s%ld", sign, (long) ip);
	} else {
		snprintf(dst, len, "%s%ld.%0*ld", sign, (long) ip, (int) decimals, (long) fp);
	}

	return dst;
}

uint32_t DebugUart_GetDroppedBytes(void) {
	return droppedBytes;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance != USART2) {
		return;
	}

	txTail = (txTail + txDmaLen) & TX_MASK;
	txBusy = false;
	TxKick();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

	// only a TX DMA error ends the transfer (RX is never started)
	if ((huart->Instance != USART2) || !txBusy || (huart->gState != HAL_UART_STATE_READY)) {
		return;
	}

	// drop the block that failed and carry on
	txTail = (txTail + txDmaLen) & TX_MASK;
	txBusy = false;
	TxKick();
}

// printf() -> debug UART (overrides the weak _write in syscalls.c)
int _write(int file, char *ptr, int len) {

	(void) file;

	if (len > 0) {
		DebugUart_Write((const uint8_t*) ptr, (uint16_t) len);
	}

	return len;
}
