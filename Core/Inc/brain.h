/*
 * brain.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Application entry points, the only thing main.c calls:
 *    brain_start() once after the CubeMX MX_xxx_Init() calls (Teensy setup())
 *    brain_loop()  forever from the while(1)                  (Teensy loop())
 */

#ifndef INC_BRAIN_H_
#define INC_BRAIN_H_

void brain_start(void);

void brain_loop(void);

#endif /* INC_BRAIN_H_ */
