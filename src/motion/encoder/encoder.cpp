/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Encoder
 * Owner: gchinellato
 */

#include <Arduino.h>
#include <math.h>
#include "encoder.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

Encoder::Encoder()
{
    ticks = 0;
    lastTicks = 0;
    cmPerTick = 0;

    setTicksPerTurn(TICKS_PER_TURN);
    setRadius(WHEEL_RADIUS);
    setCmPerTick();
}

int Encoder::getTicksPerTurn()
{
    return ticksPerTurn;
}

float Encoder::getRadius()
{
    return radius;
}

/*Get the distance (cm) from the stacionary position, checking the current ticks multiplied by one complete turn in tickes*/
float Encoder::getDistance()
{
    return ticks * cmPerTick;
}

//Set the the lenght of the wheel, C=2*pi*radius/ticksPerTurn
void Encoder::setCmPerTick()
{
    if(ticksPerTurn > 0)
    {
        cmPerTick = (2*M_PI*radius/ticksPerTurn);
    }
}

/*Set radius of the wheels in cm*/
void Encoder::setRadius(float radius)
{
    this->radius = radius;
}

/*Set the number of tick to complete one turn*/
void Encoder::setTicksPerTurn(int ticksPerTurn)
{
    this->ticksPerTurn = ticksPerTurn;
}

void Encoder::resetTicks()
{
    this->ticks = 0;
}