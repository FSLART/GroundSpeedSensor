/*
 * serial_stream.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Measurement output on the debug UART, replaces the Serial.print() calls of
 *  the Teensy loop(). Mode selected with SERIAL_STREAM_MODE in app_config.h.
 *  SERIAL_STREAM_IMAGE keeps the format of Scripts/image_and_displacement.py
 *  (change its serial port / baud to the USART2 adapter, 921600).
 */

#ifndef INC_SERIAL_STREAM_H_
#define INC_SERIAL_STREAM_H_

void SerialStream_Init(void);
void SerialStream_Task(void);

#endif /* INC_SERIAL_STREAM_H_ */
