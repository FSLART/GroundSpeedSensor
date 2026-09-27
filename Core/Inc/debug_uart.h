/*
 * debug_uart.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Non-blocking debug output on USART2 (DEBUG_TX / DEBUG_RX), replaces the
 *  Teensy USB Serial. Bytes go into a ring buffer that DMA1 Stream6 drains.
 *  printf() is redirected here too (_write override).
 *
 *  newlib-nano printf has no %f -> use DebugUart_FormatFloat().
 */

#ifndef INC_DEBUG_UART_H_
#define INC_DEBUG_UART_H_

#include <stdint.h>
#include <stddef.h>

void DebugUart_Init(void);

void DebugUart_Write(const uint8_t *data, uint16_t len);
int DebugUart_Printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

// Writes v with 'decimals' digits into dst, returns dst (to use straight in a printf arg)
char* DebugUart_FormatFloat(char *dst, size_t len, float v, uint8_t decimals);

uint32_t DebugUart_GetDroppedBytes(void);

#endif /* INC_DEBUG_UART_H_ */
