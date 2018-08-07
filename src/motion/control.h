/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#ifndef CONTROL_H
#define CONTROL_H

#define DATA_INTERVAL 	    10 // ms

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
	double speedPIDKp;
	double speedPIDKi;
	double speedPIDKd;
	double speedPIDOutputLowerLimit;
	double speedPIDOutputHigherLimit;
	double anglePIDAggKp;
	double anglePIDAggKi;
	double anglePIDAggKd;
	double anglePIDConKp;
	double anglePIDConKi;
	double anglePIDConKd;
	double anglePIDLowerLimit;
	double calibratedZeroAngle;
	int motor1MinimumSpeed;
	int motor2MinimumSpeed;
} Configuration;

struct UserControl {
	float steering;
	float direction;
};

void setConfiguration(Configuration configuration);
void control(void *pvParameter);

#endif