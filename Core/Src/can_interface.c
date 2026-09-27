/*
 * can_interface.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Port of Teensy/src/CAN_interface/CAN_interface.cpp
 *  (Milton Runberger & David Spaberg, CC BY-NC-SA 4.0).
 */

#include "can_interface.h"
#include "app_config.h"
#include "can_bus.h"
#include "ground_speed.h"
#include "height_sensor.h"
#include "paw3395.h"
#include "timebase.h"
#include "debug_uart.h"

#define IMAGE_PIXELS_PER_FRAME  6u
#define IMAGE_CHUNKS            ((PAW3395_FRAME_PIXELS + IMAGE_PIXELS_PER_FRAME - 1u) / IMAGE_PIXELS_PER_FRAME) // 216
#define IMAGE_MAX_QUEUED        8u      // keep the TX queue short so speed frames are not delayed

/* ================== MODULE STATE ================== */

// States
static bool writeSpeedEnabled = false;
static bool writeImageEnabled = false;

// Timer variables (ms)
static uint32_t writeSpeedInterval = CAN_SPEED_INTERVAL_DEFAULT_MS;
static uint32_t writeImageInterval = CAN_IMAGE_INTERVAL_DEFAULT_MS;
static uint32_t startTimeSpeed = 0;
static uint32_t startTimeImage = 0;

static uint8_t speedCounter = 0;

#if CAN_DEBUG_RX_PRINT
static uint32_t messageCount = 0;   // for debugging
#endif

// Image streaming
static uint8_t imageBuf[PAW3395_FRAME_PIXELS];
static bool imageActive = false;
static uint16_t imageChunk = 0;

/* ================== HELPERS ================== */

static int16_t Sat16(int32_t v) {

	if (v > INT16_MAX) {
		return INT16_MAX;
	}

	if (v < INT16_MIN) {
		return INT16_MIN;
	}

	return (int16_t) v;
}

static int32_t RoundToInt(float v) {
	return (int32_t) (v + ((v >= 0.0f) ? 0.5f : -0.5f));
}

// Function that starts and ends writing, interval 0 keeps the current one
static void SetSpeedWrite(bool value, uint8_t interval) {

	writeSpeedEnabled = value;

	if (value) {
		if (interval != 0) {
			writeSpeedInterval = interval;
		}

		startTimeSpeed = getRuntimeMs();

		// don't send counts accumulated while disabled
		int32_t dx, dy;
		GroundSpeed_TakeCounts(&dx, &dy);
	}
}

static void SetImageWrite(bool value, uint8_t interval) {

	writeImageEnabled = value;

	if (value) {
		if (interval != 0) {
			writeImageInterval = interval;
		}

		startTimeImage = getRuntimeMs();
	} else {
		imageActive = false;
	}
}

static void DebugIncomingMessage(const CanFrame_t *msg) {

#if CAN_DEBUG_RX_PRINT
	messageCount++;

	DebugUart_Printf("_Message #%lu\r\n_ID: %lu\r\nBuffer:", (unsigned long) messageCount, (unsigned long) msg->id);

	for (uint8_t i = 0; i < 8; i++) {
		DebugUart_Printf(" %u", msg->data[i]);
	}

	DebugUart_Printf("\r\n");

	if (msg->data[0] == CAN_MSG_SEND_SPEED) {
		int16_t x = (int16_t) (((uint16_t) msg->data[2] << 8) | msg->data[1]);
		int16_t y = (int16_t) (((uint16_t) msg->data[4] << 8) | msg->data[3]);
		DebugUart_Printf("Speed [x, y]: %d, %d\r\n", x, y);
	}
#else
	(void) msg;
#endif
}

/* ================== SPEED ================== */

static void WriteSpeed(void) {

	const GroundSpeed_Sample_t *s = GroundSpeed_GetLatest();
	uint8_t d[8] = { 0 };
	int16_t x;
	int16_t y;

#if CAN_SPEED_PAYLOAD_RAW_COUNTS
	int32_t cx, cy;
	GroundSpeed_TakeCounts(&cx, &cy);
	x = Sat16(cx);
	y = Sat16(cy);
#else
	x = s->valid ? Sat16(RoundToInt(s->vx_mps * 100.0f)) : 0;
	y = s->valid ? Sat16(RoundToInt(s->vy_mps * 100.0f)) : 0;
#endif

	int32_t h = RoundToInt(s->height_mm);

	d[0] = CAN_MSG_SEND_SPEED;
	d[1] = (uint8_t) (x & 0xFF);
	d[2] = (uint8_t) ((uint16_t) x >> 8);
	d[3] = (uint8_t) (y & 0xFF);
	d[4] = (uint8_t) ((uint16_t) y >> 8);
	d[5] = (!s->valid || (h < 0) || (h > 254)) ? 0xFFu : (uint8_t) h;
	d[6] = CanInterface_GetStatusBits();
	d[7] = speedCounter++;

	CAN_Bus_Send(CAN_SENSOR_ID, 8, d);
}

/* ================== IMAGE ================== */

static void WriteImage(void) {

	uint32_t now = getRuntimeMs();

	// grab a new frame when the previous one is fully queued
	if (!imageActive && ((now - startTimeImage) > writeImageInterval)) {

		startTimeImage = now;

		if (GroundSpeed_GrabFrame(imageBuf) == HAL_OK) {
			imageActive = true;
			imageChunk = 0;
		}
	}

	while (imageActive && (CAN_Bus_TxQueueUsed() < IMAGE_MAX_QUEUED)) {

		uint8_t d[8] = { 0 };
		uint32_t first = (uint32_t) imageChunk * IMAGE_PIXELS_PER_FRAME;

		d[0] = CAN_MSG_SEND_IMAGE;
		d[1] = (uint8_t) imageChunk;

		for (uint32_t i = 0; i < IMAGE_PIXELS_PER_FRAME; i++) {
			if ((first + i) < PAW3395_FRAME_PIXELS) {
				d[2 + i] = imageBuf[first + i];
			}
		}

		CAN_Bus_Send(CAN_SENSOR_ID, 8, d);

		if (++imageChunk >= IMAGE_CHUNKS) {
			imageActive = false;
		}
	}
}

/* ================== PUBLIC ================== */

void CanInterface_Init(void) {

	if (CAN_Bus_Init(CAN_SENSOR_ID) != HAL_OK) {
		DebugUart_Printf("CAN1 init FAILED\r\n");
	} else {
		DebugUart_Printf("CAN1 ok, sensor id %u\r\n", (unsigned) CAN_SENSOR_ID);
	}

	writeSpeedInterval = CAN_SPEED_INTERVAL_DEFAULT_MS;
	writeImageInterval = CAN_IMAGE_INTERVAL_DEFAULT_MS;

	SetSpeedWrite(CAN_SPEED_STREAM_AT_BOOT != 0, 0);
	SetImageWrite(false, 0);
}

void CanInterface_Process(void) {

	CanFrame_t msg;

	while (CAN_Bus_Receive(&msg)) {

		if ((msg.ide != CAN_ID_STD) || (msg.id != CAN_SENSOR_ID) || (msg.dlc < 1)) {
			continue;
		}

		uint8_t interval = (msg.dlc >= 2) ? msg.data[1] : 0;

		switch (msg.data[0]) {
		case CAN_CMD_ENABLE_SPEED:
			SetSpeedWrite(true, interval);
			break;
		case CAN_CMD_DISABLE_SPEED:
			SetSpeedWrite(false, 0);
			break;
		case CAN_CMD_ENABLE_IMAGE:
			SetImageWrite(true, interval);
			break;
		case CAN_CMD_DISABLE_IMAGE:
			SetImageWrite(false, 0);
			break;
		default:
			break;
		}

		DebugIncomingMessage(&msg);
	}
}

void CanInterface_Task(void) {

	if (writeSpeedEnabled && (getRuntimeMsDiff(startTimeSpeed) >= writeSpeedInterval)) {
		startTimeSpeed += writeSpeedInterval;

		// fell far behind (image grab, init): resync instead of bursting
		if (getRuntimeMsDiff(startTimeSpeed) >= writeSpeedInterval) {
			startTimeSpeed = getRuntimeMs();
		}

		WriteSpeed();
	}

	if (writeImageEnabled || imageActive) {
		WriteImage();
	}

	CAN_Bus_Service();
}

uint8_t CanInterface_GetStatusBits(void) {

	const GroundSpeed_Sample_t *s = GroundSpeed_GetLatest();
	uint8_t bits = 0;
	uint8_t mm = 0;

	if (PAW3395_IsOk() && s->valid) {
		bits |= CAN_STATUS_PAW_OK;
	}

	if (HeightSensor_GetRangeMm(&mm)) {
		bits |= CAN_STATUS_TOF_OK;
	}

	if (s->heightMeasured) {
		bits |= CAN_STATUS_HEIGHT_MEASURED;
	}

	if (imageActive) {
		bits |= CAN_STATUS_IMAGE_ACTIVE;
	}

	return bits;
}

bool CanInterface_IsSpeedEnabled(void) {
	return writeSpeedEnabled;
}

bool CanInterface_IsImageEnabled(void) {
	return writeImageEnabled;
}

HAL_StatusTypeDef CanInterface_SendCommand(uint8_t cmd, uint8_t interval_ms) {

	uint8_t d[8] = { 0 };

	d[0] = cmd;
	d[1] = interval_ms;

	return CAN_Bus_Send(CAN_SENSOR_ID, 8, d);
}
