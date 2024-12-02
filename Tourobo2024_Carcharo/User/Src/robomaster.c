/*
 * robomaster.c
 *
 *  Created on: Jul 13, 2024
 *      Author: Haruto HAGIWARA
 */

#include "robomaster.h"

void Robomaster_InitZero(RobomasterTypedef *Robomaster) {
	// Input torque target
	Robomaster->TargetTorque = 0;
	// Current angle
	Robomaster->Angle = 0;
	// Current angular velocity
	Robomaster->AngularVelocity = 0;
	// Input torque feedback
	Robomaster->FeedbackTorque = 0;
	// Motor Temperature
	Robomaster->MotorTemperature = 0;
	// Update Check
	Robomaster->Event = 0;
}

void Robomaster_RxCAN(RobomasterTypedef *Robomaster, uint8_t *RxData) {
	// Current angle
	Robomaster->Angle = RxData[0] >> 8 | RxData[1];
	// Current angular velocity
	Robomaster->AngularVelocity = RxData[2] >> 8 | RxData[3];
	// Input torque feedback
	Robomaster->FeedbackTorque = RxData[4] >> 8 | RxData[5];
	// Motor temperature
	Robomaster->MotorTemperature = RxData[6];
	// Update Check
	Robomaster->Event = 1;
}
