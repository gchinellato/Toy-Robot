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
	Motor(mcpwm_unit_t channel, mcpwm_timer_t timer, mcpwm_operator_t opr, mcpwm_io_signals_t signals, int pinPWM, int pinCW, int pinCCW, int pinCS);
	void setSpeedPercentage(float speed);
    void motorOff();
    void currentSense();
private:
	void motorGo(int direct, float pwm);
    mcpwm_unit_t mcpwm_num;
    mcpwm_io_signals_t io_signal;
    mcpwm_timer_t timer_num;
    mcpwm_operator_t op_num;
    int pwmpin;
    int inApin;
    int inBpin;
    int cspin;
};

#endif