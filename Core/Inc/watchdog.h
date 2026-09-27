/*
 * watchdog.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  IWDG helpers (IWDG itself is configured in the .ioc: LSI/32, reload 500 -> ~500 ms).
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_

#include <stdint.h>

typedef enum {
	RESET_CAUSE_UNKNOWN = 0,
	RESET_CAUSE_POWER_ON,
	RESET_CAUSE_PIN,
	RESET_CAUSE_SOFTWARE,
	RESET_CAUSE_IWDG,
	RESET_CAUSE_WWDG,
	RESET_CAUSE_LOW_POWER,
	RESET_CAUSE_BROWNOUT
} ResetCause_t;

void Watchdog_Init(void);
void Watchdog_Kick(void);

ResetCause_t Watchdog_GetResetCause(void);
const char* Watchdog_ResetCauseName(ResetCause_t cause);

#endif /* INC_WATCHDOG_H_ */
