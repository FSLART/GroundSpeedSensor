/*
 * status_led.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  LED_HEARTBEAT blinks while brain_loop() runs, LED_FAULT is on while a
 *  sensor or the CAN bus has a problem.
 */

#ifndef INC_STATUS_LED_H_
#define INC_STATUS_LED_H_

#include <stdbool.h>

void StatusLed_Init(void);
void StatusLed_Task(void);
void StatusLed_SetFault(bool fault);

#endif /* INC_STATUS_LED_H_ */
