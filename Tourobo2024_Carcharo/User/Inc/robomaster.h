/*
 * robomaster.h
 *
 *  Created on: Jul 13, 2024
 *      Author: Haruto HAGIWARA
 */

#ifndef INC_ROBOMASTER_H_
#define INC_ROBOMASTER_H_

#include "stm32f7xx_hal.h"

typedef struct {
	// Input torque target
	int16_t TargetTorque;
	// Current angle
	int16_t Angle;
	// Current angular velocity
	int16_t AngularVelocity;
	// Input torque feedback
	int16_t FeedbackTorque;
	// Motor Temperature
	uint8_t MotorTemperature;

	// Update Check
	uint32_t Event;

	// Encoder FeedBack
	float EncoderAngularVelocity;

	// Target Angular Velocity
	float TargetAngularVelocity;
	float PreTargetAngularVelocity;
	// Angular Velocity Error
	float AngularVelocityError;
	// Pre Angular Velocity Error
	float PreAngularVelocityError;
	// Integral
	float Integral;
} RobomasterTypedef;

void Robomaster_InitZero(RobomasterTypedef *Robomaster);

void Robomaster_RxCAN(RobomasterTypedef *Robomaster, uint8_t *RxData);

#endif /* INC_ROBOMASTER_H_ */
