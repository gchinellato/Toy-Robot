/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: PID
 * Owner: gchinellato
 */

#include "PID.h"

PID::PID()
{
    this->Ci=0;
    this->lastTime=0;
    this->lastError=0;
}

float PID::compute(float input)
{
    /* Performs a PID computation and returns a control value based on
    the elapsed time (dt) and the error signal from a summing junction
    (the error parameter)*/
    unsigned long now = 0;//millis();
    float dt;
    float error;
    float de;
    float output;

    /* Calculate delta time (seconds) */
    dt = (float)(now - lastTime)/1000.0f;

    /* Calculate delta error */
    error = setpoint - input;
   
    de = error - lastError;

    /* Proportional Term */
    Cp = error;

    /* Integral Term */
    Ci += error*dt;

    Cd = 0;
    /* to avoid division by zero */
    if(dt>0)
    {
        /* Derivative term */
        Cd = de/dt;
    }

    /* Save for the next iteration */
    lastError = error;
    lastTime = now;

    /* Sum terms: pTerm+iTerm+dTerm */
    output = Cp*Kp + Ci*Ki + Cd*Kd;

    /* Saturation - Windup guard for Integral term do not reach very large values */
    if(output > WINDUP_GUARD){
        output = WINDUP_GUARD;
    }
    else if (output < -WINDUP_GUARD){
        output = -WINDUP_GUARD;
    }

    return output;
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