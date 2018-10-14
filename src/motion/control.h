/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "pid/PID.h"
#include "motion/encoder/encoder.h"

#define DATA_INTERVAL 	    7 // ms

enum cmd{
	STARTED = 0,
	DIRECTION,
	STEERING,
	SPEED_PID,
	ANGLE_PID,
	HEADING_PID,
	ZERO_ANGLE,
	ANGLE_LIMITE,
	CF_IMU
};

typedef struct configuration {
	float speedPIDKp;
	float speedPIDKi;
	float speedPIDKd;
	float speedPIDOutputLowerLimit;
	float speedPIDOutputHigherLimit;
	float headingPIDKp;
	float headingPIDKi;
	float headingPIDKd;
	float anglePIDConKp;
	float anglePIDConKi;
	float anglePIDConKd;
	float anglePIDLowerLimit;
	float calibratedZeroAngle;
	boolean started;
	float cf;
	float steering;
	float direction;
} Configuration;

extern Configuration gConfig;
extern Encoder encoder1;
extern Encoder encoder2;

void setConfiguration(Configuration configuration);
void control(void *pvParameter);

#endif