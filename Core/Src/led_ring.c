/*
 * led_ring.c
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  TIM1 runs PWM continuously at 800 kHz (96 MHz / 120). One DMA transfer
 *  writes one CCR1 value per bit. The last value is 0, so after the transfer
 *  the line just stays low (= WS2812 latch/reset) without stopping the timer.
 */

#include "led_ring.h"
#include "app_config.h"
#include "tim.h"
#include "timebase.h"

#define WS2812_PERIOD_TICKS     120u                            // TIM1 ARR + 1 (1.25 us)
#define WS2812_T0H_TICKS        38u                             // 0.40 us
#define WS2812_T1H_TICKS        77u                             // 0.80 us
#define WS2812_LEAD_SLOTS       2u                              // low before the data
#define WS2812_TAIL_SLOTS       2u                              // low after the data (CCR back to 0)
#define WS2812_LATCH_US         300u                            // min low time between frames (WS2812B V5)
#define WS2812_BITS             (LED_RING_NUM_PIXELS * 24u)
#define WS2812_BUF_LEN          (WS2812_LEAD_SLOTS + WS2812_BITS + WS2812_TAIL_SLOTS)

static uint8_t pixels[LED_RING_NUM_PIXELS][3];  // r, g, b
static uint8_t brightness = 0;                  // 0 = full (Adafruit style: stored as brightness + 1)
static uint16_t pwmBuf[WS2812_BUF_LEN];
static volatile bool busy = false;
static volatile uint32_t lastDoneUs = 0;

static void LedRing_DmaDone(DMA_HandleTypeDef *hdma) {

	(void) hdma;

	__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	lastDoneUs = Timebase_Micros();
	busy = false;
}

static void LedRing_DmaError(DMA_HandleTypeDef *hdma) {
	LedRing_DmaDone(hdma);
}

static uint8_t Scale(uint8_t c) {

	if (brightness == 0) {
		return c;
	}

	return (uint8_t) (((uint16_t) c * brightness) >> 8);
}

void LedRing_Init(void) {

	for (uint16_t i = 0; i < LED_RING_NUM_PIXELS; i++) {
		pixels[i][0] = 0;
		pixels[i][1] = 0;
		pixels[i][2] = 0;
	}

	brightness = 0;
	busy = false;

	// output low, timer running forever
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

#if LED_RING_ON_AT_BOOT
	LedRing_Fill(LED_RING_BOOT_R, LED_RING_BOOT_G, LED_RING_BOOT_B);
	LedRing_SetBrightness(LED_RING_BOOT_BRIGHTNESS);
	LedRing_Show();
#else
	LedRing_Show();     // all off, puts the ring in a known state
#endif
}

void LedRing_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {

	if (index >= LED_RING_NUM_PIXELS) {
		return;
	}

	pixels[index][0] = r;
	pixels[index][1] = g;
	pixels[index][2] = b;
}

void LedRing_Fill(uint8_t r, uint8_t g, uint8_t b) {

	for (uint16_t i = 0; i < LED_RING_NUM_PIXELS; i++) {
		LedRing_SetPixel(i, r, g, b);
	}
}

void LedRing_SetBrightness(uint8_t b) {
	brightness = (uint8_t) (b + 1u);   // 255 -> 0 = no scaling
}

bool LedRing_IsBusy(void) {
	return busy;
}

bool LedRing_Show(void) {

	if (busy) {
		return false;
	}

	// latch: line must stay low for a while after the previous frame
	while ((Timebase_Micros() - lastDoneUs) < WS2812_LATCH_US) {
	}

	uint32_t n = 0;

	for (uint32_t i = 0; i < WS2812_LEAD_SLOTS; i++) {
		pwmBuf[n++] = 0;
	}

	for (uint16_t p = 0; p < LED_RING_NUM_PIXELS; p++) {

		// WS2812 order: G, R, B, MSB first
		uint8_t grb[3] = { Scale(pixels[p][1]), Scale(pixels[p][0]), Scale(pixels[p][2]) };

		for (uint8_t c = 0; c < 3; c++) {
			for (int8_t bit = 7; bit >= 0; bit--) {
				pwmBuf[n++] = (grb[c] & (1u << bit)) ? WS2812_T1H_TICKS : WS2812_T0H_TICKS;
			}
		}
	}

	for (uint32_t i = 0; i < WS2812_TAIL_SLOTS; i++) {
		pwmBuf[n++] = 0;
	}

	DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID_CC1];

	hdma->XferCpltCallback = LedRing_DmaDone;
	hdma->XferErrorCallback = LedRing_DmaError;

	busy = true;

	if (HAL_DMA_Start_IT(hdma, (uint32_t) pwmBuf, (uint32_t) &htim1.Instance->CCR1, WS2812_BUF_LEN) != HAL_OK) {
		busy = false;
		return false;
	}

	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);

	return true;
}

void LedRing_TurnOn(void) {

	LedRing_Fill(255, 255, 255);    // pixels.fill(0x00FFFFFF, 0, 24)
	LedRing_SetBrightness(255);
	LedRing_Show();
}

void LedRing_TurnOff(void) {

	LedRing_Fill(0, 0, 0);
	LedRing_Show();
}
