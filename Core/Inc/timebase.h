/*
 * timebase.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Time helpers. TIM2 runs free at 1 MHz (32 bit) -> microsecond timestamps
 *  and delays (Teensy micros()/delayMicroseconds()). Milliseconds come from
 *  the HAL SysTick (Teensy millis()/delay()).
 */

#ifndef INC_TIMEBASE_H_
#define INC_TIMEBASE_H_

#include <stdint.h>

void Timebase_Init(void);

uint32_t Timebase_Micros(void);
void Timebase_DelayUs(uint32_t us);

uint32_t getRuntimeMs(void);
uint32_t getRuntimeMsDiff(uint32_t startTime);

#endif /* INC_TIMEBASE_H_ */
