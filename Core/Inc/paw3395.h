/*
 * paw3395.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  PixArt PAW3395DM-T6QU optical displacement sensor on SPI1 (mode 3, 6 MHz).
 *  Pins: PAW_NCS, PAW_SCLK, PAW_MISO, PAW_MOSI, PAW_NRESET, PAW_MOTION.
 *
 *  Port of Teensy/src/PAW3395DM_T6QU_Driver/Sensor_Driver.cpp
 *  (Milton Runberger & David Spaberg, CC BY-NC-SA 4.0).
 */

#ifndef INC_PAW3395_H_
#define INC_PAW3395_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define PAW3395_PRODUCT_ID          0x51u
#define PAW3395_FRAME_WIDTH         36u
#define PAW3395_FRAME_PIXELS        (PAW3395_FRAME_WIDTH * PAW3395_FRAME_WIDTH) // 1296

/* Registers used by this driver */
#define PAW3395_REG_PRODUCT_ID      0x00u
#define PAW3395_REG_MOTION          0x02u
#define PAW3395_REG_DELTA_X_L       0x03u
#define PAW3395_REG_DELTA_X_H       0x04u
#define PAW3395_REG_DELTA_Y_L       0x05u
#define PAW3395_REG_DELTA_Y_H       0x06u
#define PAW3395_REG_SQUAL           0x07u
#define PAW3395_REG_POWER_UP_RESET  0x3Au
#define PAW3395_REG_SET_RESOLUTION  0x47u
#define PAW3395_REG_RESOLUTION_X_L  0x48u
#define PAW3395_REG_RESOLUTION_X_H  0x49u
#define PAW3395_REG_RESOLUTION_Y_L  0x4Au
#define PAW3395_REG_RESOLUTION_Y_H  0x4Bu
#define PAW3395_REG_RAWDATA_GRAB    0x58u
#define PAW3395_REG_RAWDATA_STATUS  0x59u
#define PAW3395_REG_BANK_SELECT     0x7Fu

typedef struct {
	uint8_t motion;     // Motion register (reading it latches the deltas)
	int16_t dx;         // counts since last read
	int16_t dy;
	uint8_t squal;      // surface quality
} PAW3395_Motion_t;

HAL_StatusTypeDef PAW3395_Init(void);          // power up + init register sequence (init_chip)
bool PAW3395_IsOk(void);
uint8_t PAW3395_GetProductId(void);

void PAW3395_WriteReg(uint8_t reg, uint8_t data);   // set_register
uint8_t PAW3395_ReadReg(uint8_t reg);               // read_register

HAL_StatusTypeDef PAW3395_ReadMotion(PAW3395_Motion_t *m);
HAL_StatusTypeDef PAW3395_GrabFrame(uint8_t *frame);    // raw_data(), PAW3395_FRAME_PIXELS bytes

void PAW3395_LiftCutoff2mm(void);               // lift_cutoff_2mm
void PAW3395_SetResolution(uint16_t value);     // set_CPI (raw register value)

#endif /* INC_PAW3395_H_ */
