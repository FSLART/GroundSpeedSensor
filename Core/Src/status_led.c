/*
 * status_led.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 */

#include "status_led.h"
#include "app_config.h"
#include "main.h"
#include "timebase.h"

static uint32_t lastToggle = 0;

void StatusLed_Init(void) {

	HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
	lastToggle = getRuntimeMs();
}

void StatusLed_Task(void) {

	if (getRuntimeMsDiff(lastToggle) >= HEARTBEAT_PERIOD_MS) {
		lastToggle = getRuntimeMs();
		HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
	}
}

void StatusLed_SetFault(bool fault) {
	HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, fault ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
