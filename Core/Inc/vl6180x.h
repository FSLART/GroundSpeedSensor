/*
 * vl6180x.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  ST VL6180X ToF range + ambient light sensor on I2C1 (TOF_SCL / TOF_SDA),
 *  chip enable on TOF_CE. Replaces the Adafruit_VL6180X library used by the
 *  Teensy firmware. Same init settings (ST AN4545) and lux math, but the
 *  measurements are split in start / poll / read so nothing blocks the loop.
 */

#ifndef INC_VL6180X_H_
#define INC_VL6180X_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define VL6180X_I2C_ADDR        (0x29u << 1)
#define VL6180X_MODEL_ID        0xB4u

/* ALS gains (Adafruit_VL6180X values) */
#define VL6180X_ALS_GAIN_1      0x06u
#define VL6180X_ALS_GAIN_1_25   0x05u
#define VL6180X_ALS_GAIN_1_67   0x04u
#define VL6180X_ALS_GAIN_2_5    0x03u
#define VL6180X_ALS_GAIN_5      0x02u
#define VL6180X_ALS_GAIN_10     0x01u
#define VL6180X_ALS_GAIN_20     0x00u
#define VL6180X_ALS_GAIN_40     0x07u

/* Range status (RESULT__RANGE_STATUS >> 4) */
#define VL6180X_ERROR_NONE      0u

HAL_StatusTypeDef VL6180X_Init(void);

HAL_StatusTypeDef VL6180X_StartRange(void);
bool VL6180X_IsRangeReady(void);
HAL_StatusTypeDef VL6180X_ReadRange(uint8_t *range_mm, uint8_t *status);

HAL_StatusTypeDef VL6180X_StartAls(uint8_t gain);
bool VL6180X_IsAlsReady(void);
HAL_StatusTypeDef VL6180X_ReadLux(uint8_t gain, float *lux);

HAL_StatusTypeDef VL6180X_Write8(uint16_t reg, uint8_t value);
HAL_StatusTypeDef VL6180X_Read8(uint16_t reg, uint8_t *value);
HAL_StatusTypeDef VL6180X_Read16(uint16_t reg, uint16_t *value);

#endif /* INC_VL6180X_H_ */
