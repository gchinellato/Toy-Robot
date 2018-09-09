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

#define DATA_INTERVAL 	    20 // ms

enum cmd{
	STARTED = 0,
	DIRECTION,
	STEERING,
	SPEED_PID,
	ANGLE_PID_AGGR,
	ANGLE_PID_CONS,
	ZERO_ANGLE,
	ANGLE_LIMITE
};

typedef struct configuration {
	float speedPIDKp;
	float speedPIDKi;
	float speedPIDKd;
	float speedPIDOutputLowerLimit;
	float speedPIDOutputHigherLimit;
	float anglePIDAggKp;
	float anglePIDAggKi;
	float anglePIDAggKd;
	float anglePIDConKp;
	float anglePIDConKi;
	float anglePIDConKd;
	float anglePIDLowerLimit;
	float calibratedZeroAngle;
	boolean started;
	float steering;
	float direction;
} Configuration;

extern Configuration gConfig;

void setConfiguration(Configuration configuration);
void control(void *pvParameter);

#endif