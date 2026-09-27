/*
 * ground_speed.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Port of Sensor_Driver::get_speed() (Milton Runberger & David Spaberg, CC BY-NC-SA 4.0).
 */

#include "ground_speed.h"
#include "app_config.h"
#include "paw3395.h"
#include "height_sensor.h"
#include "timebase.h"
#include "debug_uart.h"

static GroundSpeed_Sample_t latest = { 0 };

static uint32_t lastSampleUs = 0;
static int32_t accumDx = 0;
static int32_t accumDy = 0;
static bool filterPrimed = false;

float GroundSpeed_HeightScale(float height_mm) {

#if GSS_SCALE_MODE == GSS_SCALE_POLY
	return (GSS_POLY_A * height_mm * height_mm) + (GSS_POLY_B * height_mm) + GSS_POLY_C;
#else
	return height_mm * GSS_LINEAR_GAIN;
#endif
}

static float GetHeightMm(bool *measured) {

	*measured = false;

#if GSS_HEIGHT_SOURCE == GSS_HEIGHT_MEASURED
	uint8_t mm = 0;

	if (HeightSensor_GetRangeMm(&mm) && (mm > 0)) {
		*measured = true;
		return (float) mm;
	}
#endif

	return GSS_HEIGHT_MM;
}

void GroundSpeed_Init(void) {

	if (PAW3395_Init() == HAL_OK) {

#if GSS_LIFT_CUTOFF_2MM
		PAW3395_LiftCutoff2mm();
#endif
		DebugUart_Printf("PAW3395 ok (product id 0x%02X)\r\n", PAW3395_GetProductId());

	} else {
		DebugUart_Printf("PAW3395 NOT found (product id 0x%02X, expected 0x%02X)\r\n", PAW3395_GetProductId(), PAW3395_PRODUCT_ID);
	}

	lastSampleUs = Timebase_Micros();
}

void GroundSpeed_Task(void) {

	uint32_t now = Timebase_Micros();
	uint32_t dt_us = now - lastSampleUs;

	if (dt_us < GSS_SAMPLE_PERIOD_US) {
		return;
	}

	lastSampleUs = now;

	PAW3395_Motion_t m;

	if ((PAW3395_ReadMotion(&m) != HAL_OK) || !PAW3395_IsOk()) {
		latest.valid = false;
		return;
	}

	bool measured = false;
	float height = GetHeightMm(&measured);
	float scale = GroundSpeed_HeightScale(height);
	float dt_ms = (float) dt_us / 1000.0f;

	// Speed = Scaling * Displacement / Time   (mm / ms = m / s)
	float vx = scale * ((float) m.dx * GSS_MM_PER_COUNT) / dt_ms;
	float vy = scale * ((float) m.dy * GSS_MM_PER_COUNT) / dt_ms;

	if (!filterPrimed || !latest.valid) {
		latest.vx_mps = vx;
		latest.vy_mps = vy;
		filterPrimed = true;
	} else {
		latest.vx_mps += GSS_SPEED_FILTER_ALPHA * (vx - latest.vx_mps);
		latest.vy_mps += GSS_SPEED_FILTER_ALPHA * (vy - latest.vy_mps);
	}

	latest.dx = m.dx;
	latest.dy = m.dy;
	latest.squal = m.squal;
	latest.height_mm = height;
	latest.heightMeasured = measured;
	latest.dt_us = dt_us;
	latest.timestamp_ms = getRuntimeMs();
	latest.sampleCount++;
	latest.valid = true;

	accumDx += m.dx;
	accumDy += m.dy;
}

const GroundSpeed_Sample_t* GroundSpeed_GetLatest(void) {
	return &latest;
}

void GroundSpeed_TakeCounts(int32_t *dx, int32_t *dy) {

	*dx = accumDx;
	*dy = accumDy;
	accumDx = 0;
	accumDy = 0;
}

HAL_StatusTypeDef GroundSpeed_GrabFrame(uint8_t *frame) {

	HAL_StatusTypeDef st = PAW3395_GrabFrame(frame);

	// counts during the grab are lost: clear the motion registers and restart dt
	PAW3395_Motion_t m;
	PAW3395_ReadMotion(&m);
	lastSampleUs = Timebase_Micros();

	return st;
}
