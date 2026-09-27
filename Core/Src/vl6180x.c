/*
 * vl6180x.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Port of what Teensy/src/Height_Sensor/Height_sensor.cpp used from
 *  Adafruit_VL6180X (begin, readRange, readLux).
 */

#include "vl6180x.h"
#include "i2c.h"

#define VL6180X_I2C_TIMEOUT_MS                      5u

#define VL6180X_REG_IDENTIFICATION_MODEL_ID         0x000u
#define VL6180X_REG_SYSTEM_INTERRUPT_CONFIG         0x014u
#define VL6180X_REG_SYSTEM_INTERRUPT_CLEAR          0x015u
#define VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET       0x016u
#define VL6180X_REG_SYSRANGE_START                  0x018u
#define VL6180X_REG_SYSRANGE_INTERMEASUREMENT       0x01Bu
#define VL6180X_REG_SYSALS_START                    0x038u
#define VL6180X_REG_SYSALS_ANALOGUE_GAIN            0x03Fu
#define VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI    0x040u
#define VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO    0x041u
#define VL6180X_REG_RESULT_RANGE_STATUS             0x04Du
#define VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO    0x04Fu
#define VL6180X_REG_RESULT_ALS_VAL                  0x050u
#define VL6180X_REG_RESULT_RANGE_VAL                0x062u

#define VL6180X_ALS_INTEGRATION_MS                  100u

/* ================== I2C ================== */

HAL_StatusTypeDef VL6180X_Write8(uint16_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1, VL6180X_I2C_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &value, 1, VL6180X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef VL6180X_Read8(uint16_t reg, uint8_t *value) {
	return HAL_I2C_Mem_Read(&hi2c1, VL6180X_I2C_ADDR, reg, I2C_MEMADD_SIZE_16BIT, value, 1, VL6180X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef VL6180X_Read16(uint16_t reg, uint16_t *value) {

	uint8_t buf[2];
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, VL6180X_I2C_ADDR, reg, I2C_MEMADD_SIZE_16BIT, buf, 2, VL6180X_I2C_TIMEOUT_MS);

	*value = ((uint16_t) buf[0] << 8) | buf[1];
	return st;
}

/* ================== INIT ================== */

// ST AN4545 recommended settings (Adafruit_VL6180X::loadSettings)
static const struct {
	uint16_t reg;
	uint8_t val;
} vlSettings[] = {
	/* private registers */
	{ 0x0207, 0x01 }, { 0x0208, 0x01 }, { 0x0096, 0x00 }, { 0x0097, 0xFD },
	{ 0x00E3, 0x00 }, { 0x00E4, 0x04 }, { 0x00E5, 0x02 }, { 0x00E6, 0x01 },
	{ 0x00E7, 0x03 }, { 0x00F5, 0x02 }, { 0x00D9, 0x05 }, { 0x00DB, 0xCE },
	{ 0x00DC, 0x03 }, { 0x00DD, 0xF8 }, { 0x009F, 0x00 }, { 0x00A3, 0x3C },
	{ 0x00B7, 0x00 }, { 0x00BB, 0x3C }, { 0x00B2, 0x09 }, { 0x00CA, 0x09 },
	{ 0x0198, 0x01 }, { 0x01B0, 0x17 }, { 0x01AD, 0x00 }, { 0x00FF, 0x05 },
	{ 0x0100, 0x05 }, { 0x0199, 0x05 }, { 0x01A6, 0x1B }, { 0x01AC, 0x3E },
	{ 0x01A7, 0x1F }, { 0x0030, 0x00 },
	/* public registers */
	{ 0x0011, 0x10 },   // enable polling for 'new sample ready' when measurement completes
	{ 0x010A, 0x30 },   // averaging sample period
	{ 0x003F, 0x46 },   // light and dark gain
	{ 0x0031, 0xFF },   // auto calibration every 255 range measurements
	{ 0x0041, 0x63 },   // ALS integration time 100 ms
	{ 0x002E, 0x01 },   // single temperature calibration of the ranging sensor
	/* optional public registers */
	{ VL6180X_REG_SYSRANGE_INTERMEASUREMENT, 0x09 },    // 100 ms (continuous mode only)
	{ 0x003E, 0x31 },   // ALS inter measurement period 500 ms (continuous mode only)
	{ 0x0014, 0x24 },   // interrupt on 'new sample ready' for range and ALS
};

HAL_StatusTypeDef VL6180X_Init(void) {

	uint8_t id = 0;

	// chip enable low -> high = fresh boot (also after an MCU-only reset)
	HAL_GPIO_WritePin(TOF_CE_GPIO_Port, TOF_CE_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TOF_CE_GPIO_Port, TOF_CE_Pin, GPIO_PIN_SET);
	HAL_Delay(2);

	if (VL6180X_Read8(VL6180X_REG_IDENTIFICATION_MODEL_ID, &id) != HAL_OK) {
		return HAL_ERROR;
	}

	if (id != VL6180X_MODEL_ID) {
		return HAL_ERROR;
	}

	for (uint32_t i = 0; i < (sizeof(vlSettings) / sizeof(vlSettings[0])); i++) {
		if (VL6180X_Write8(vlSettings[i].reg, vlSettings[i].val) != HAL_OK) {
			return HAL_ERROR;
		}
	}

	return VL6180X_Write8(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
}

/* ================== RANGE ================== */

HAL_StatusTypeDef VL6180X_StartRange(void) {

	uint8_t st = 0;

	// device must be ready for a range measurement
	if (VL6180X_Read8(VL6180X_REG_RESULT_RANGE_STATUS, &st) != HAL_OK) {
		return HAL_ERROR;
	}

	if ((st & 0x01u) == 0) {
		return HAL_BUSY;
	}

	return VL6180X_Write8(VL6180X_REG_SYSRANGE_START, 0x01);
}

bool VL6180X_IsRangeReady(void) {

	uint8_t st = 0;

	if (VL6180X_Read8(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO, &st) != HAL_OK) {
		return false;
	}

	return (st & 0x04u) != 0;
}

HAL_StatusTypeDef VL6180X_ReadRange(uint8_t *range_mm, uint8_t *status) {

	uint8_t st = 0;

	if (VL6180X_Read8(VL6180X_REG_RESULT_RANGE_VAL, range_mm) != HAL_OK) {
		return HAL_ERROR;
	}

	if (VL6180X_Read8(VL6180X_REG_RESULT_RANGE_STATUS, &st) != HAL_OK) {
		return HAL_ERROR;
	}

	*status = st >> 4;

	return VL6180X_Write8(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);
}

/* ================== AMBIENT LIGHT ================== */

HAL_StatusTypeDef VL6180X_StartAls(uint8_t gain) {

	uint8_t reg = 0;

	if (VL6180X_Read8(VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, &reg) != HAL_OK) {
		return HAL_ERROR;
	}

	reg = (uint8_t) ((reg & ~0x38u) | (0x4u << 3)); // IRQ on ALS ready

	if (gain > VL6180X_ALS_GAIN_40) {
		gain = VL6180X_ALS_GAIN_40;
	}

	if ((VL6180X_Write8(VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, reg) != HAL_OK)
			|| (VL6180X_Write8(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI, 0) != HAL_OK)
			|| (VL6180X_Write8(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO, VL6180X_ALS_INTEGRATION_MS) != HAL_OK)
			|| (VL6180X_Write8(VL6180X_REG_SYSALS_ANALOGUE_GAIN, 0x40u | gain) != HAL_OK)) {
		return HAL_ERROR;
	}

	return VL6180X_Write8(VL6180X_REG_SYSALS_START, 0x01);
}

bool VL6180X_IsAlsReady(void) {

	uint8_t st = 0;

	if (VL6180X_Read8(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO, &st) != HAL_OK) {
		return false;
	}

	// "new sample ready threshold event"
	return ((st >> 3) & 0x07u) == 4u;
}

HAL_StatusTypeDef VL6180X_ReadLux(uint8_t gain, float *lux) {

	uint16_t raw = 0;

	if (VL6180X_Read16(VL6180X_REG_RESULT_ALS_VAL, &raw) != HAL_OK) {
		return HAL_ERROR;
	}

	if (VL6180X_Write8(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07) != HAL_OK) {
		return HAL_ERROR;
	}

	float l = (float) raw * 0.32f;    // calibrated count/lux

	switch (gain) {
	case VL6180X_ALS_GAIN_1:
		break;
	case VL6180X_ALS_GAIN_1_25:
		l /= 1.25f;
		break;
	case VL6180X_ALS_GAIN_1_67:
		l /= 1.67f;
		break;
	case VL6180X_ALS_GAIN_2_5:
		l /= 2.5f;
		break;
	case VL6180X_ALS_GAIN_5:
		l /= 5.0f;
		break;
	case VL6180X_ALS_GAIN_10:
		l /= 10.0f;
		break;
	case VL6180X_ALS_GAIN_20:
		l /= 20.0f;
		break;
	case VL6180X_ALS_GAIN_40:
		l /= 40.0f;
		break;
	default:
		break;
	}

	l *= 100.0f;
	l /= (float) VL6180X_ALS_INTEGRATION_MS;

	*lux = l;
	return HAL_OK;
}
