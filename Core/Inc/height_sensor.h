/*
 * height_sensor.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Ride height (sensor -> ground) and ambient light from the VL6180X.
 *  Port of Teensy/src/Height_Sensor/Height_sensor.cpp, but non-blocking:
 *  HeightSensor_Task() runs a small state machine from brain_loop().
 */

#ifndef INC_HEIGHT_SENSOR_H_
#define INC_HEIGHT_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

void HeightSensor_Init(void);      // init_heightSensor
void HeightSensor_Task(void);

bool HeightSensor_IsOk(void);
bool HeightSensor_GetRangeMm(uint8_t *range_mm);   // height_measurement, false if stale/invalid
bool HeightSensor_GetLux(float *lux);              // lux_value

#endif /* INC_HEIGHT_SENSOR_H_ */
