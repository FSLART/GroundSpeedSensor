/*
 * brain.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Ground speed sensor application (STM32F412RET6).
 *  Port of Teensy/src/main.cpp from "Design and Prototyping of an Optical
 *  Non-contact Ground Speed Sensor for a Race Car" (Chalmers, 2025),
 *  original code by Milton Runberger & David Spaberg, CC BY-NC-SA 4.0:
 *  https://github.com/spaberg/Optical-Ground-Speed-Sensor
 *
 *  Nothing in brain_loop() blocks for long: every module keeps its own
 *  timing and returns right away when there's nothing to do.
 */

#include "main.h"
#include "brain.h"
#include "app_config.h"

#include "timebase.h"
#include "watchdog.h"
#include "debug_uart.h"
#include "status_led.h"
#include "ground_speed.h"
#include "height_sensor.h"
#include "can_interface.h"
#include "can_bus.h"
#include "led_ring.h"
#include "serial_stream.h"
#include "paw3395.h"

#define FAULT_CHECK_PERIOD_MS   100u

static uint32_t lastFaultCheck = 0;

static void FaultCheck(void) {

	if (getRuntimeMsDiff(lastFaultCheck) < FAULT_CHECK_PERIOD_MS) {
		return;
	}

	lastFaultCheck = getRuntimeMs();

	bool fault = false;

	if (!PAW3395_IsOk() || !GroundSpeed_GetLatest()->valid) {
		fault = true;
	}

#if HEIGHT_SENSOR_ENABLE
	if (!HeightSensor_IsOk()) {
		fault = true;
	}
#endif

	if (!CAN_Bus_IsStarted() || CAN_Bus_IsBusOff()) {
		fault = true;
	}

	StatusLed_SetFault(fault);
}

void brain_start(void) {

	Timebase_Init();
	Watchdog_Init();
	DebugUart_Init();
	StatusLed_Init();

	DebugUart_Printf("\r\n=== Ground Speed Sensor (STM32F412) ===\r\n");
	DebugUart_Printf("Reset cause: %s\r\n", Watchdog_ResetCauseName(Watchdog_GetResetCause()));

	// Power up initialization for the PAW sensor (+ lift cutoff 2 mm)
	GroundSpeed_Init();
	Watchdog_Kick();

	// Initialization of height sensor
	HeightSensor_Init();
	Watchdog_Kick();

	// NeoPixel ring (turn_on_pixel_ring)
	LedRing_Init();

	// CAN: filter, start, speed stream on/off
	CanInterface_Init();

	SerialStream_Init();
	Watchdog_Kick();

	DebugUart_Printf("Setup complete\r\n");
}

void brain_loop(void) {

	Watchdog_Kick();

	// CAN commands in (read_msg)
	CanInterface_Process();

	// displacement -> speed, every GSS_SAMPLE_PERIOD_US
	GroundSpeed_Task();

	// ride height + lux, non-blocking
	HeightSensor_Task();

	// CAN frames out (write_messages)
	CanInterface_Task();

	// debug UART output
	SerialStream_Task();

	FaultCheck();
	StatusLed_Task();
}
