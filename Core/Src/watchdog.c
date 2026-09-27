/*
 * watchdog.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 */

#include "watchdog.h"
#include "iwdg.h"

static ResetCause_t resetCause = RESET_CAUSE_UNKNOWN;

void Watchdog_Init(void) {

	uint32_t csr = RCC->CSR;

	// order matters: every reset also sets PINRSTF, a POR also sets BORRSTF
	if (csr & RCC_CSR_IWDGRSTF) {
		resetCause = RESET_CAUSE_IWDG;
	} else if (csr & RCC_CSR_WWDGRSTF) {
		resetCause = RESET_CAUSE_WWDG;
	} else if (csr & RCC_CSR_SFTRSTF) {
		resetCause = RESET_CAUSE_SOFTWARE;
	} else if (csr & RCC_CSR_LPWRRSTF) {
		resetCause = RESET_CAUSE_LOW_POWER;
	} else if (csr & RCC_CSR_PORRSTF) {
		resetCause = RESET_CAUSE_POWER_ON;
	} else if (csr & RCC_CSR_BORRSTF) {
		resetCause = RESET_CAUSE_BROWNOUT;
	} else if (csr & RCC_CSR_PINRSTF) {
		resetCause = RESET_CAUSE_PIN;
	}

	__HAL_RCC_CLEAR_RESET_FLAGS();

	// stop the IWDG while the core is halted by the debugger
	__HAL_DBGMCU_FREEZE_IWDG();

	Watchdog_Kick();
}

void Watchdog_Kick(void) {
	HAL_IWDG_Refresh(&hiwdg);
}

ResetCause_t Watchdog_GetResetCause(void) {
	return resetCause;
}

const char* Watchdog_ResetCauseName(ResetCause_t cause) {

	switch (cause) {
	case RESET_CAUSE_POWER_ON:
		return "power on";
	case RESET_CAUSE_PIN:
		return "NRST pin";
	case RESET_CAUSE_SOFTWARE:
		return "software";
	case RESET_CAUSE_IWDG:
		return "IWDG";
	case RESET_CAUSE_WWDG:
		return "WWDG";
	case RESET_CAUSE_LOW_POWER:
		return "low power";
	case RESET_CAUSE_BROWNOUT:
		return "brownout";
	default:
		return "unknown";
	}
}
