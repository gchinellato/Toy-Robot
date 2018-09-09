/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Motor
 * Owner: gchinellato
 */

#include <Arduino.h>
#include "driver/mcpwm.h"
#include "motor.h"

Motor::Motor(mcpwm_unit_t channel, mcpwm_timer_t timer, mcpwm_operator_t opr, mcpwm_io_signals_t signals, int pinPWM, int pinCW, int pinCCW, int pinCS)
{
    mcpwm_num = channel; // MCPWM Channel(0-1)
    io_signal = signals; // MCPWM signal
    timer_num = timer; // MCPWM timer
    op_num = opr; // MCPWM timer
    pwmpin = pinPWM; // PWM input
    inApin = pinCW;  // INA: Clockwise input
    inBpin = pinCCW; // INB: Counter-clockwise input
    cspin = pinCS; // CS: Current sense ANALOG input

    /* Initialize digital pins as outputs */
    pinMode(inApin, OUTPUT);
    pinMode(inBpin, OUTPUT);

    /* Initialize braked */
    digitalWrite(inApin, LOW);
    digitalWrite(inBpin, LOW);

    mcpwm_gpio_init(channel, signals, pwmpin);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 pwm: should be a value between ? and 255, higher the number, the faster it'll go
 */
void Motor::motorGo(int direct, float pwm)
{
    if (direct <= BRAKEGND)
    {
      /* Set inA */
      if (direct <= CW)
        digitalWrite(inApin, HIGH);
      else
        digitalWrite(inApin, LOW);

      /* Set inB */
      if ((direct==BRAKEVCC)||(direct==CCW))
        digitalWrite(inBpin, HIGH);
      else
        digitalWrite(inBpin, LOW);

        //mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
        mcpwm_set_duty(mcpwm_num, timer_num, op_num, pwm);
        mcpwm_set_duty_type(mcpwm_num, timer_num, op_num, MCPWM_DUTY_MODE_0);
    }
}

/* set speed in percentage from -100 to 100 */
void Motor::setSpeedPercentage(float speed)
{
    /* anything above 100 or below -100 is invalid */
    if (speed > 100)
        speed = 100;
    else if (speed < -100)
        speed = -100;

    /* negative speed */
    if (speed > 0) {
        motorGo(CW, (255/100 * speed));
    }
    else if (speed < 0){
        motorGo(CCW, (-255/100 * speed));
    }
    else {
        motorOff();
    }
}

void Motor::motorOff()
{
    /* Initialize braked */
    digitalWrite(inApin, LOW);
    digitalWrite(inBpin, LOW);
    mcpwm_set_signal_low(mcpwm_num, timer_num, op_num);
}

void Motor::currentSense()
{

}