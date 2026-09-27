/*
 * timebase.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 */

#include "timebase.h"
#include "tim.h"

void Timebase_Init(void) {

	// TIM2: prescaler 96-1 -> 1 MHz, period 0xFFFFFFFF (see .ioc)
	HAL_TIM_Base_Start(&htim2);
}

uint32_t Timebase_Micros(void) {
	return __HAL_TIM_GET_COUNTER(&htim2);
}

// Blocking, waits at least 'us' microseconds (+1 tick so a short delay never collapses to 0)
void Timebase_DelayUs(uint32_t us) {

	uint32_t start = Timebase_Micros();

	while ((Timebase_Micros() - start) <= us) {
	}
}

uint32_t getRuntimeMs(void) {
	return HAL_GetTick();
}

uint32_t getRuntimeMsDiff(uint32_t startTime) {
	return HAL_GetTick() - startTime;
}
