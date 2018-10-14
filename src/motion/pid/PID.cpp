/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: PID
 * Owner: gchinellato
 */

#include <Arduino.h>
#include "PID.h"

PID::PID()
{
    this->Ci=0;
    this->lastTime=0;
    this->lastError=0;
    setTunings(0,0,0);
}

float PID::compute(float input)
{
    /* Performs a PID computation and returns a control value based on
    the elapsed time (dt) and the error signal from a summing junction
    (the error parameter) */
    unsigned long now = millis();
    float dt;
    float error;
    float de;
    float di;
    float output;
    float outputSat;

    /* Calculate delta time in seconds */
    dt = (float)(now - lastTime)/1000.0f;

    /* Calculate error and delta error */
    error = setpoint - input;   
    de = error - lastError;
    di = input - lastInput;

    /* Proportional Term */
    Cp = error*Kp;

    /* Integral Term */
    Ci += error*Ki*dt;

    /* Derivative term */
    Cd = 0;
    if(dt>0){
       Cd = (di*Kd)/dt;
    }

    /* Sum terms: pTerm+iTerm+dTerm */
    output = Cp + Ci + Cd;

    /* Saturation - Windup guard for Integral term do not reach very large values */
    if(output > WINDUP_GUARD){
        outputSat = WINDUP_GUARD;
    }
    else if (output < -WINDUP_GUARD){
        outputSat = -WINDUP_GUARD;
    }
    else{
        outputSat = output;
    }

    /* Save for the next iteration */
    lastError = error;
    lastInput = input;
    lastTime = now;

    return outputSat;
}

void PID::setSetpoint(float value)
{
    this->setpoint = value;
}

float PID::getSetpoint()
{
    return setpoint;
}

void PID::setTunings(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}
