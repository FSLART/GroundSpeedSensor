/*
 * ground_speed.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Speed over ground from the PAW3395 displacement + height.
 *  Port of Sensor_Driver::get_speed() / print_displacement(): here the sensor
 *  is read ONCE per sample and every consumer (CAN, UART) uses that sample,
 *  so nobody steals counts from the others (the Teensy read the motion
 *  registers in several places, each read clears them).
 *
 *  speed [m/s] = scale(height) * counts * GSS_MM_PER_COUNT / dt [ms]
 */

#ifndef INC_GROUND_SPEED_H_
#define INC_GROUND_SPEED_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	int16_t dx;             // counts in this sample
	int16_t dy;
	float vx_mps;           // speed, filtered with GSS_SPEED_FILTER_ALPHA
	float vy_mps;
	float height_mm;        // height used for scaling
	bool heightMeasured;    // true = VL6180X, false = GSS_HEIGHT_MM
	uint8_t squal;          // PAW surface quality
	uint32_t dt_us;
	uint32_t timestamp_ms;
	uint32_t sampleCount;   // +1 every new sample
	bool valid;             // last read OK
} GroundSpeed_Sample_t;

void GroundSpeed_Init(void);
void GroundSpeed_Task(void);

const GroundSpeed_Sample_t* GroundSpeed_GetLatest(void);

// Counts accumulated since the previous call (Teensy CAN write_speed payload)
void GroundSpeed_TakeCounts(int32_t *dx, int32_t *dy);

// Raw 36x36 image. Navigation stops while grabbing (~30 ms), timing is restarted after.
HAL_StatusTypeDef GroundSpeed_GrabFrame(uint8_t *frame);

float GroundSpeed_HeightScale(float height_mm);

#endif /* INC_GROUND_SPEED_H_ */
