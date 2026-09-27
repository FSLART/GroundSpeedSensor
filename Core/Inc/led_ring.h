/*
 * led_ring.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  24x WS2812 (NeoPixel) ring on LED_RING_DIN (TIM1_CH1 PWM, 800 kHz, DMA2 Stream1).
 *  Replaces Adafruit_NeoPixel from the Teensy firmware (turn_on_pixel_ring).
 *  WS2812 wants 5 V logic: use a level shifter / 5 V pull-up on DIN if needed.
 */

#ifndef INC_LED_RING_H_
#define INC_LED_RING_H_

#include <stdbool.h>
#include <stdint.h>

void LedRing_Init(void);

void LedRing_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void LedRing_Fill(uint8_t r, uint8_t g, uint8_t b);
void LedRing_SetBrightness(uint8_t brightness);
bool LedRing_Show(void);            // false if the previous frame is still being sent
bool LedRing_IsBusy(void);

void LedRing_TurnOn(void);          // turn_on_pixel_ring(): all white, full brightness
void LedRing_TurnOff(void);

#endif /* INC_LED_RING_H_ */
