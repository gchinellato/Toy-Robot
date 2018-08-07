/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Motor
 * Owner: gchinellato
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <driver/dac.h>

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

#define VOLTAGE_MIN 0
#define VOLTAGE_MAX 3.3

#define DEAD_BAND 8

class Motor
{
public:
	Motor(dac_channel_t channel);
	void setSpeedPercentage(float speed);
    void motorOff();
    void currentSense();
	float motorSpeed;
private:
	void motorGo(int direct, float pwm);
    dac_channel_t dac_num;
};

#endif