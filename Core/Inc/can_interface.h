/*
 * can_interface.h
 *
 *  Created on: Sep 27, 2026
 *      Author: jpser
 *
 *  Ground speed sensor CAN protocol, see app_config.h for the frame layout.
 *  Port of Teensy/src/CAN_interface/CAN_interface.cpp
 *  (Milton Runberger & David Spaberg, CC BY-NC-SA 4.0).
 */

#ifndef INC_CAN_INTERFACE_H_
#define INC_CAN_INTERFACE_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

void CanInterface_Init(void);
void CanInterface_Process(void);    // read_msg: handle received commands
void CanInterface_Task(void);       // write_messages: periodic speed / image frames

uint8_t CanInterface_GetStatusBits(void);
bool CanInterface_IsSpeedEnabled(void);
bool CanInterface_IsImageEnabled(void);

/* Send a command to another sensor with the same protocol (write_start_sending_speed,
 * write_start_sending_image, write_set_speed_interval, write_set_image_interval) */
HAL_StatusTypeDef CanInterface_SendCommand(uint8_t cmd, uint8_t interval_ms);

#endif /* INC_CAN_INTERFACE_H_ */
