/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: PID
 * Owner: gchinellato
 */

#ifndef PID_H
#define PID_H

#define SPEED_SETPOINT		0.0
#define SPEED_KP 			0.8
#define SPEED_KI 			0.0
#define SPEED_KD 			0.0
#define ANGLE_SETPOINT 		0.0
#define ANGLE_LIMIT 		45.0
#define HEADING_KP 	    	0.0
#define HEADING_KI   		0.0
#define HEADING_KD 		    0.0
#define ANGLE_KP_CONS 		4.5
#define ANGLE_KI_CONS 		0.01
#define ANGLE_KD_CONS 		0.0
#define ANGLE_IRRECOVERABLE 45.0
#define CALIBRATED_ZERO_ANGLE 11.0
#define WINDUP_GUARD 		100

#define N 20

enum PIDTuning {
    CONSERVATIVE,
    AGGRESSIVE
};

class PID
{
public:
	PID();
	float compute (float input);
    void setSetpoint(float setpoint);
    float getSetpoint();
    void setTunings(float Kp, float Ki, float Kd);
private:
	float lastError;
    float lastInput;
	unsigned long lastTime;
    float setpoint;
    float Cp;
    float Ci;
    float Cd;
    float Kp;
    float Ki;
    float Kd;
};

#endif