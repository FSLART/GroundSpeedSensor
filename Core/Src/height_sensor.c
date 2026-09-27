/*
 * height_sensor.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 */

#include "height_sensor.h"
#include "app_config.h"
#include "vl6180x.h"
#include "timebase.h"
#include "debug_uart.h"

#define HS_POLL_PERIOD_MS       1u      // don't hammer the I2C bus while waiting
#define HS_RANGE_TIMEOUT_MS     50u
#define HS_ALS_TIMEOUT_MS       (100u + 100u)
#define HS_REINIT_PERIOD_MS     1000u
#define HS_MAX_ERRORS           5u      // consecutive errors before re-init

typedef enum {
	HS_STATE_IDLE,
	HS_STATE_RANGE_WAIT,
	HS_STATE_ALS_WAIT,
	HS_STATE_OFFLINE
} HsState_t;

static HsState_t state = HS_STATE_OFFLINE;

static uint32_t stateStart = 0;
static uint32_t lastRangeStart = 0;
static uint32_t lastPoll = 0;
static uint32_t lastInitTry = 0;
static uint8_t rangeCounter = 0;
static uint8_t errorCount = 0;

static uint8_t rangeMm = 0;
static uint8_t rangeStatus = 0xFF;
static uint32_t rangeTime = 0;
static bool rangeEverRead = false;

static float lux = 0.0f;
static bool luxValid = false;

static void HS_TryInit(void) {

	lastInitTry = getRuntimeMs();

	if (VL6180X_Init() == HAL_OK) {
		state = HS_STATE_IDLE;
		errorCount = 0;
		DebugUart_Printf("VL6180X height sensor ok\r\n");
	} else {
		state = HS_STATE_OFFLINE;
	}
}

static void HS_Error(void) {

	state = HS_STATE_IDLE;

	if (++errorCount >= HS_MAX_ERRORS) {
		state = HS_STATE_OFFLINE;
		DebugUart_Printf("VL6180X lost, retrying\r\n");
	}
}

void HeightSensor_Init(void) {

	if (!HEIGHT_SENSOR_ENABLE) {
		return;
	}

	HS_TryInit();

	if (state == HS_STATE_OFFLINE) {
		// Teensy hung here with while(1); keep running without height instead
		DebugUart_Printf("Failed to initalize proximity sensor\r\n");
	}
}

void HeightSensor_Task(void) {

	if (!HEIGHT_SENSOR_ENABLE) {
		return;
	}

	uint32_t now = getRuntimeMs();

	switch (state) {

	case HS_STATE_OFFLINE:

		if ((now - lastInitTry) >= HS_REINIT_PERIOD_MS) {
			HS_TryInit();
		}
		break;

	case HS_STATE_IDLE:

		if ((now - lastRangeStart) < HEIGHT_RANGE_PERIOD_MS) {
			break;
		}

		lastRangeStart = now;

		if (++rangeCounter >= HEIGHT_ALS_EVERY_N_RANGES) {

			rangeCounter = 0;

			if (VL6180X_StartAls(HEIGHT_ALS_GAIN) == HAL_OK) {
				state = HS_STATE_ALS_WAIT;
				stateStart = now;
			} else {
				HS_Error();
			}

		} else {

			HAL_StatusTypeDef st = VL6180X_StartRange();

			if (st == HAL_OK) {
				state = HS_STATE_RANGE_WAIT;
				stateStart = now;
			} else if (st != HAL_BUSY) {
				HS_Error();
			}
		}
		break;

	case HS_STATE_RANGE_WAIT:

		if ((now - lastPoll) < HS_POLL_PERIOD_MS) {
			break;
		}
		lastPoll = now;

		if (VL6180X_IsRangeReady()) {

			uint8_t mm = 0;
			uint8_t status = 0;

			if (VL6180X_ReadRange(&mm, &status) == HAL_OK) {
				rangeMm = mm;
				rangeStatus = status;
				rangeTime = now;
				rangeEverRead = true;
				errorCount = 0;
				state = HS_STATE_IDLE;
			} else {
				HS_Error();
			}

		} else if ((now - stateStart) > HS_RANGE_TIMEOUT_MS) {
			HS_Error();
		}
		break;

	case HS_STATE_ALS_WAIT:

		if ((now - lastPoll) < HS_POLL_PERIOD_MS) {
			break;
		}
		lastPoll = now;

		if (VL6180X_IsAlsReady()) {

			float l = 0.0f;

			if (VL6180X_ReadLux(HEIGHT_ALS_GAIN, &l) == HAL_OK) {
				lux = l;
				luxValid = true;
				errorCount = 0;
				state = HS_STATE_IDLE;
			} else {
				HS_Error();
			}

		} else if ((now - stateStart) > HS_ALS_TIMEOUT_MS) {
			HS_Error();
		}
		break;

	default:
		state = HS_STATE_OFFLINE;
		break;
	}
}

bool HeightSensor_IsOk(void) {
	return state != HS_STATE_OFFLINE;
}

bool HeightSensor_GetRangeMm(uint8_t *range_mm) {

	*range_mm = rangeMm;

	if (!rangeEverRead || (state == HS_STATE_OFFLINE)) {
		return false;
	}

	if (rangeStatus != VL6180X_ERROR_NONE) {
		return false;
	}

	return getRuntimeMsDiff(rangeTime) <= HEIGHT_VALID_TIMEOUT_MS;
}

bool HeightSensor_GetLux(float *l) {

	*l = lux;
	return luxValid && (state != HS_STATE_OFFLINE);
}
