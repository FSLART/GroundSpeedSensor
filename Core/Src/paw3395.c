/*
 * paw3395.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Port of Teensy/src/PAW3395DM_T6QU_Driver/Sensor_Driver.cpp
 *  (Milton Runberger & David Spaberg, CC BY-NC-SA 4.0).
 */

#include "paw3395.h"
#include "spi.h"
#include "timebase.h"
#include "watchdog.h"

/* SPI timings [us] (Sensor_Driver.cpp t_sww, t_swr, t_srad) */
#define PAW_T_SWW_US        5u      // after a write, before the next access
#define PAW_T_SRR_US        5u      // after a read, before the next access
#define PAW_T_SRAD_US       2u      // read: address -> data
#define PAW_T_SCLK_NCS_US   1u      // write: last clock -> NCS high

#define PAW_SPI_TIMEOUT_MS  2u

#define PAW_GRAB_START_TIMEOUT_US   50000u  // wait for run mode / first pixel
#define PAW_GRAB_PIXEL_TIMEOUT_US   2000u   // wait for each next pixel

static bool pawDetected = false;   // product ID matched at init
static bool pawOk = false;         // detected and last access without SPI error
static bool spiError = false;
static uint8_t productId = 0;

/* Power up initialization register setting (init_chip), same order as the Teensy driver */
static const uint8_t pawInitSeq[][2] = {
	{ 0x7F, 0x07 }, { 0x40, 0x41 }, { 0x7F, 0x00 }, { 0x40, 0x80 },
	{ 0x7F, 0x0E }, { 0x55, 0x0D }, { 0x56, 0x1B }, { 0x57, 0xE8 }, { 0x58, 0xD5 },
	{ 0x7F, 0x14 }, { 0x42, 0xBC }, { 0x43, 0x74 }, { 0x4B, 0x20 }, { 0x4D, 0x00 }, { 0x53, 0x0E },
	{ 0x7F, 0x05 }, { 0x44, 0x04 }, { 0x4D, 0x06 }, { 0x51, 0x40 }, { 0x53, 0x40 }, { 0x55, 0xCA },
	{ 0x5A, 0xE8 }, { 0x5B, 0xEA }, { 0x61, 0x31 }, { 0x62, 0x64 }, { 0x6D, 0xB8 }, { 0x6E, 0x0F },
	{ 0x70, 0x02 }, { 0x4A, 0x2A }, { 0x60, 0x26 },
	{ 0x7F, 0x06 }, { 0x6D, 0x70 }, { 0x6E, 0x60 }, { 0x6F, 0x04 }, { 0x53, 0x02 }, { 0x55, 0x11 },
	{ 0x7A, 0x01 }, { 0x7D, 0x51 },
	{ 0x7F, 0x07 }, { 0x41, 0x10 }, { 0x42, 0x32 }, { 0x43, 0x00 },
	{ 0x7F, 0x08 }, { 0x71, 0x4F },
	{ 0x7F, 0x09 }, { 0x62, 0x1F }, { 0x63, 0x1F }, { 0x65, 0x03 }, { 0x66, 0x03 }, { 0x67, 0x1F },
	{ 0x68, 0x1F }, { 0x69, 0x03 }, { 0x6A, 0x03 }, { 0x6C, 0x1F }, { 0x6D, 0x1F }, { 0x51, 0x04 },
	{ 0x53, 0x20 }, { 0x54, 0x20 }, { 0x71, 0x0C }, { 0x72, 0x07 }, { 0x73, 0x07 },
	{ 0x7F, 0x0A }, { 0x4A, 0x14 }, { 0x4C, 0x14 }, { 0x55, 0x19 },
	{ 0x7F, 0x14 }, { 0x4B, 0x30 }, { 0x4C, 0x03 }, { 0x61, 0x0B }, { 0x62, 0x0A }, { 0x63, 0x02 },
	{ 0x7F, 0x15 }, { 0x4C, 0x02 }, { 0x56, 0x02 }, { 0x41, 0x91 }, { 0x4D, 0x0A },
	{ 0x7F, 0x00 },
};

/* ================== LOW LEVEL ================== */

static inline void PAW_Select(void) {
	HAL_GPIO_WritePin(PAW_NCS_GPIO_Port, PAW_NCS_Pin, GPIO_PIN_RESET);
}

static inline void PAW_Deselect(void) {
	HAL_GPIO_WritePin(PAW_NCS_GPIO_Port, PAW_NCS_Pin, GPIO_PIN_SET);
}

static uint8_t PAW_Transfer(uint8_t tx) {

	uint8_t rx = 0;

	if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, PAW_SPI_TIMEOUT_MS) != HAL_OK) {
		spiError = true;
	}

	return rx;
}

void PAW3395_WriteReg(uint8_t reg, uint8_t data) {

	PAW_Select();
	PAW_Transfer(reg | 0x80u);
	PAW_Transfer(data);
	Timebase_DelayUs(PAW_T_SCLK_NCS_US);
	PAW_Deselect();
	Timebase_DelayUs(PAW_T_SWW_US);
}

uint8_t PAW3395_ReadReg(uint8_t reg) {

	PAW_Select();
	PAW_Transfer(reg & 0x7Fu);
	Timebase_DelayUs(PAW_T_SRAD_US);
	uint8_t value = PAW_Transfer(0x00);
	PAW_Deselect();
	Timebase_DelayUs(PAW_T_SRR_US);

	return value;
}

/* ================== INIT ================== */

HAL_StatusTypeDef PAW3395_Init(void) {

	spiError = false;

	// hardware reset (Teensy only did the soft reset, NRESET is harmless if tied high)
	HAL_GPIO_WritePin(PAW_NRESET_GPIO_Port, PAW_NRESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(PAW_NRESET_GPIO_Port, PAW_NRESET_Pin, GPIO_PIN_SET);

	HAL_Delay(50);

	// NCS high -> low -> high resets the SPI port
	PAW_Deselect();
	Timebase_DelayUs(10);
	PAW_Select();
	Timebase_DelayUs(10);
	PAW_Deselect();

	// power_up_reset register
	PAW3395_WriteReg(PAW3395_REG_POWER_UP_RESET, 0x5A);
	HAL_Delay(5);

	for (uint32_t i = 0; i < (sizeof(pawInitSeq) / sizeof(pawInitSeq[0])); i++) {
		PAW3395_WriteReg(pawInitSeq[i][0], pawInitSeq[i][1]);
	}

	HAL_Delay(1);
	Watchdog_Kick();

	// wait until register 0x6C = 0x80 (max 60 times, checking every 1 ms)
	for (int i = 0; i < 60; i++) {

		if (PAW3395_ReadReg(0x6C) == 0x80) {
			break;
		} else if (i == 59) {
			PAW3395_WriteReg(0x7F, 0x14);
			PAW3395_WriteReg(0x6C, 0x00);
			PAW3395_WriteReg(0x7F, 0x00);
		}

		HAL_Delay(1);
	}

	PAW3395_WriteReg(0x22, 0x00);
	PAW3395_WriteReg(0x55, 0x00);
	PAW3395_WriteReg(0x7F, 0x07);
	PAW3395_WriteReg(0x40, 0x40);
	PAW3395_WriteReg(0x7F, 0x00);
	PAW3395_WriteReg(0x68, 0x01);

	// clear motion registers
	PAW3395_ReadReg(PAW3395_REG_MOTION);
	PAW3395_ReadReg(PAW3395_REG_DELTA_X_L);
	PAW3395_ReadReg(PAW3395_REG_DELTA_X_H);
	PAW3395_ReadReg(PAW3395_REG_DELTA_Y_L);
	PAW3395_ReadReg(PAW3395_REG_DELTA_Y_H);

	productId = PAW3395_ReadReg(PAW3395_REG_PRODUCT_ID);

	pawDetected = (!spiError) && (productId == PAW3395_PRODUCT_ID);
	pawOk = pawDetected;

	return pawOk ? HAL_OK : HAL_ERROR;
}

bool PAW3395_IsOk(void) {
	return pawOk;
}

uint8_t PAW3395_GetProductId(void) {
	return productId;
}

/* ================== MOTION ================== */

HAL_StatusTypeDef PAW3395_ReadMotion(PAW3395_Motion_t *m) {

	spiError = false;

	m->motion = PAW3395_ReadReg(PAW3395_REG_MOTION);   // latches the deltas

	uint8_t xl = PAW3395_ReadReg(PAW3395_REG_DELTA_X_L);
	uint8_t xh = PAW3395_ReadReg(PAW3395_REG_DELTA_X_H);
	uint8_t yl = PAW3395_ReadReg(PAW3395_REG_DELTA_Y_L);
	uint8_t yh = PAW3395_ReadReg(PAW3395_REG_DELTA_Y_H);

	m->squal = PAW3395_ReadReg(PAW3395_REG_SQUAL);

	m->dx = (int16_t) (((uint16_t) xh << 8) | xl);
	m->dy = (int16_t) (((uint16_t) yh << 8) | yl);

	// ok again as soon as reads succeed, as long as the chip was found at init
	pawOk = pawDetected && !spiError;

	return spiError ? HAL_ERROR : HAL_OK;
}

/* ================== RAW FRAME ================== */

static bool PAW_WaitReg(uint8_t reg, uint8_t mask, uint8_t value, uint32_t timeoutUs) {

	uint32_t start = Timebase_Micros();

	while ((PAW3395_ReadReg(reg) & mask) != value) {
		if ((Timebase_Micros() - start) > timeoutUs) {
			return false;
		}
	}

	return true;
}

HAL_StatusTypeDef PAW3395_GrabFrame(uint8_t *frame) {

	HAL_StatusTypeDef status = HAL_OK;

	spiError = false;

	PAW3395_WriteReg(PAW3395_REG_BANK_SELECT, 0x00);
	PAW3395_WriteReg(0x40, 0x80);

	// wait until bit 0 and 1 of Motion are 0 (run mode)
	if (!PAW_WaitReg(PAW3395_REG_MOTION, 0x03, 0x00, PAW_GRAB_START_TIMEOUT_US)) {
		status = HAL_TIMEOUT;
	}

	if (status == HAL_OK) {

		PAW3395_WriteReg(0x50, 0x01);
		PAW3395_WriteReg(0x55, 0x04);
		PAW3395_WriteReg(PAW3395_REG_RAWDATA_GRAB, 0xFF);

		if (!PAW_WaitReg(PAW3395_REG_RAWDATA_STATUS, 0xC0, 0xC0, PAW_GRAB_START_TIMEOUT_US)) {
			status = HAL_TIMEOUT;
		} else {
			frame[0] = PAW3395_ReadReg(PAW3395_REG_RAWDATA_GRAB);
		}
	}

	for (uint32_t i = 1; (i < PAW3395_FRAME_PIXELS) && (status == HAL_OK); i++) {

		if (!PAW_WaitReg(PAW3395_REG_RAWDATA_STATUS, 0x80, 0x80, PAW_GRAB_PIXEL_TIMEOUT_US)) {
			status = HAL_TIMEOUT;
			break;
		}

		frame[i] = PAW3395_ReadReg(PAW3395_REG_RAWDATA_GRAB);
	}

	// back to navigation, also after a timeout
	PAW3395_WriteReg(0x40, 0x00);
	PAW3395_WriteReg(0x50, 0x00);
	PAW3395_WriteReg(0x55, 0x00);

	if (spiError) {
		status = HAL_ERROR;
	}

	return status;
}

/* ================== SETTINGS ================== */

void PAW3395_LiftCutoff2mm(void) {

	PAW3395_WriteReg(PAW3395_REG_BANK_SELECT, 0x0C);
	PAW3395_WriteReg(0x4E, 0x02);
	PAW3395_WriteReg(PAW3395_REG_BANK_SELECT, 0x00);
}

void PAW3395_SetResolution(uint16_t value) {

	uint8_t low = (uint8_t) (value & 0xFFu);
	uint8_t high = (uint8_t) (value >> 8);

	PAW3395_WriteReg(PAW3395_REG_RESOLUTION_X_L, low);
	PAW3395_WriteReg(PAW3395_REG_RESOLUTION_X_H, high);
	PAW3395_WriteReg(PAW3395_REG_RESOLUTION_Y_L, low);
	PAW3395_WriteReg(PAW3395_REG_RESOLUTION_Y_H, high);
	PAW3395_WriteReg(PAW3395_REG_SET_RESOLUTION, 0x01);
}
