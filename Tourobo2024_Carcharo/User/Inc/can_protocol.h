/*
 * can_protocol.h
 *
 *  Created on: Jul 17, 2024
 *      Author: Haruto HAGIWARA
 */

#ifndef INC_CAN_PROTOCOL_H_
#define INC_CAN_PROTOCOL_H_

#include "stm32f7xx_hal.h"

typedef enum{
	ROBOMASTER_ID = 0x200,
	ACTZ_ID = 0x400,
	SHOKI_ID = 0x300,
} CAN_ID;


typedef enum{
	ROBOMASTER_MASK = 0x7F0,
	ACTZ_MASK = 0x7F0,
	SHOKI_MASK = 0x7F0,
} CAN_MASK;

#endif /* INC_CAN_PROTOCOL_H_ */
