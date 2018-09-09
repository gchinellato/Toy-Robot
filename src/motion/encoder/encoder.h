/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Encoder
 * Owner: gchinellato
 */

#ifndef ENCODER_H
#define ENCODER_H

#define WHEEL_RADIUS 6.0 //cm

/* 64 of motor shaft * 29 (29:1) gearbox = 1856 counter per revolution with gearbox when both A and B channels and rising and falling edges */
/* 16 of motor shaft * 29 (29:1) gearbox = 464 counter per revolution with gearbox when only one channel either rising or falling edges */
#define TICKS_PER_TURN 464

void encoderISR1();
void encoderISR2();

class Encoder
{
public:
	Encoder();
    int getTicksPerTurn();
    float getRadius();
    float getDistance();
    void resetTicks();
    void setTicksPerTurn(int ticksPerTurn);
    void setRadius(float radius);
    void setCmPerTick();
    volatile long ticks;
	long lastTicks;
private:
    int ticksPerTurn;
    float radius;
    float cmPerTick;
	float velocity;
};

#endif