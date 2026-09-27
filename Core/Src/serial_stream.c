/*
 * serial_stream.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 */

#include "serial_stream.h"
#include "app_config.h"
#include "ground_speed.h"
#include "height_sensor.h"
#include "paw3395.h"
#include "timebase.h"
#include "debug_uart.h"

#include <stdio.h>

static uint32_t lastSampleCount = 0;

#if SERIAL_STREAM_MODE == SERIAL_STREAM_IMAGE
static uint8_t frame[PAW3395_FRAME_PIXELS];
static uint32_t lastImage = 0;

// print_raw_data() + print_displacement()
static void StreamImage(void) {

	if (getRuntimeMsDiff(lastImage) < SERIAL_STREAM_IMAGE_PERIOD_MS) {
		return;
	}

	lastImage = getRuntimeMs();

	if (GroundSpeed_GrabFrame(frame) != HAL_OK) {
		return;
	}

	char buf[5 * 36 + 4];

	for (uint32_t row = 0; row < PAW3395_FRAME_WIDTH; row++) {

		int n = 0;

		for (uint32_t col = 0; col < PAW3395_FRAME_WIDTH; col++) {
			uint32_t i = row * PAW3395_FRAME_WIDTH + col;
			bool last = (i == (PAW3395_FRAME_PIXELS - 1u));
			n += snprintf(&buf[n], sizeof(buf) - n, last ? "%u\r\n" : "%u,", frame[i]);
		}

		DebugUart_Write((const uint8_t*) buf, (uint16_t) n);
	}

	const GroundSpeed_Sample_t *s = GroundSpeed_GetLatest();
	DebugUart_Printf("%d,%d\r\n", s->dx, s->dy);
}
#endif

void SerialStream_Init(void) {

	lastSampleCount = GroundSpeed_GetLatest()->sampleCount;

#if SERIAL_STREAM_MODE == SERIAL_STREAM_CSV
	DebugUart_Printf("t_ms,dx,dy,vx_mps,vy_mps,h_mm,range_mm,lux,squal\r\n");
#endif
}

void SerialStream_Task(void) {

#if SERIAL_STREAM_MODE == SERIAL_STREAM_IMAGE
	StreamImage();
#elif SERIAL_STREAM_MODE != SERIAL_STREAM_OFF

	const GroundSpeed_Sample_t *s = GroundSpeed_GetLatest();

	// one line per new speed sample
	if (s->sampleCount == lastSampleCount) {
		return;
	}

	lastSampleCount = s->sampleCount;

#if SERIAL_STREAM_MODE == SERIAL_STREAM_DISPLACEMENT
	DebugUart_Printf("%d,%d\r\n", s->dx, s->dy);

#elif SERIAL_STREAM_MODE == SERIAL_STREAM_SPEED_X
	char vx[16];
	DebugUart_Printf("%s\r\n", DebugUart_FormatFloat(vx, sizeof(vx), s->vx_mps, 2));

#elif SERIAL_STREAM_MODE == SERIAL_STREAM_CSV
	char vx[16], vy[16], h[16], lx[16];
	uint8_t range = 0;
	float lux = 0.0f;
	bool rangeOk = HeightSensor_GetRangeMm(&range);
	bool luxOk = HeightSensor_GetLux(&lux);

	DebugUart_Printf("%lu,%d,%d,%s,%s,%s,%d,%s,%u\r\n",
			(unsigned long) s->timestamp_ms,
			s->dx,
			s->dy,
			DebugUart_FormatFloat(vx, sizeof(vx), s->vx_mps, 3),
			DebugUart_FormatFloat(vy, sizeof(vy), s->vy_mps, 3),
			DebugUart_FormatFloat(h, sizeof(h), s->height_mm, 0),
			rangeOk ? (int) range : -1,
			luxOk ? DebugUart_FormatFloat(lx, sizeof(lx), lux, 2) : "-1",
			s->squal);
#endif

#endif
}
