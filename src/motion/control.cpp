/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#include <stdio.h>
#include <string.h>
#include <Wire.h>
#include "driver/mcpwm.h"
#include "control.h"
#include "pinmux/pinmux.h"
#include "imu/imu.h"
#include "motion/motor/motor.h"
#include "motion/encoder/encoder.h"
#include "../comm/serial/serial.h"
#include "../main.h"

/* PID objects */
PID speedPID;
PID anglePID;
PID headingPID;

/* Motor objects */
Motor motor1(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM0A, PWM1_PIN, CW1_PIN, CCW1_PIN, CS1_PIN);
Motor motor2(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM0B, PWM2_PIN, CW2_PIN, CCW2_PIN, CS2_PIN);

/* Encoder objects */
Encoder encoder1;
Encoder encoder2;

portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;

/* interrupt functions for counting revolutions in the encoders.
 * When the callback function is called due an interrup event on pinEncoderAx
 * and pinEncoderBx=true, then is clockwise, if not it is counter-clockwise
 */
void IRAM_ATTR encoderISR1()
{
    portENTER_CRITICAL_ISR(&mux1);
    if(digitalRead(ENCODERB1_PIN))
    { encoder1.ticks++; }
    else
    { encoder1.ticks--; }
    portEXIT_CRITICAL_ISR(&mux1);
}

void IRAM_ATTR encoderISR2()
{
    portENTER_CRITICAL_ISR(&mux2);
    if(digitalRead(ENCODERB2_PIN))
    { encoder2.ticks++; }
    else
    { encoder2.ticks--; }
    portEXIT_CRITICAL_ISR(&mux2);
}

/* Configuration 
 * global struct, read-write multithread
 */
Configuration gConfig;

void setConfiguration(Configuration *configuration)
{
    configuration->speedPIDKp = SPEED_KP;
    configuration->speedPIDKi = SPEED_KI;
    configuration->speedPIDKd = SPEED_KD;
    configuration->speedPIDOutputLowerLimit = -10.00;
    configuration->speedPIDOutputHigherLimit = 10.00;
    configuration->headingPIDKp = HEADING_KP;
    configuration->headingPIDKi = HEADING_KI;
    configuration->headingPIDKd = HEADING_KD;
    configuration->anglePIDConKp = ANGLE_KP_CONS;
    configuration->anglePIDConKi = ANGLE_KI_CONS;
    configuration->anglePIDConKd = ANGLE_KD_CONS;
    configuration->anglePIDLowerLimit = ANGLE_LIMIT;
    configuration->calibratedZeroAngle = CALIBRATED_ZERO_ANGLE;
    configuration->started = false;
    configuration->cf = 0.9785;
    configuration->steering = 0;
    configuration->direction = 0;
}

void control(void *pvParameter)
{
    /* I2C Init */  
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    /* Interrupt Init */
    pinMode(ENCODERA1_PIN, INPUT_PULLUP);
    pinMode(ENCODERB1_PIN, INPUT);
    pinMode(ENCODERA2_PIN, INPUT_PULLUP);    
    pinMode(ENCODERB2_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODERA1_PIN), encoderISR1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODERA2_PIN), encoderISR2, RISING);

    /* PMM Init */
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 5000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    /* IMY Init */
    GY80 imu;

    vTaskDelay(50);

    unsigned long timestamp=0;
    unsigned long timestamp_old=0;
    float dt=0;
    float velocity1, velocity2;
    float distance1, distance2;
    float lastDistance1=0, lastDistance2=0;
    float *ori;
    float speedPIDInput, anglePIDInput, headingPIDInput;
    float speedPIDOutput, anglePIDOutput, headingPIDOutput;
    
    char sendBuffer[BUFFER_MAX];
  
    setConfiguration(&gConfig);

    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        timestamp = millis();
        if ((timestamp - timestamp_old) >= DATA_INTERVAL)
        {
            /* convert from ms to sec */
            dt = (float)(timestamp - timestamp_old)/1000.0f; 
            timestamp_old = timestamp;

            ori = imu.getOrientation(1, gConfig.cf, dt);
            anglePIDInput = ori[1];

            /* getSpeed */
            distance1 = encoder1.getDistance();
            distance2 = -encoder2.getDistance();
            velocity1 = distance1 - lastDistance1;
            velocity2 = distance2 - lastDistance2;
            lastDistance1 = distance1;
            lastDistance2 = distance2;
          
            /* Compute Speed PID (input is wheel speed and output is angleSetpoint, direction max speed = 6 / position = 200 ticks) */
            speedPIDInput = (velocity1+velocity2)/2;
            speedPID.setSetpoint(gConfig.direction);
            speedPID.setTunings(gConfig.speedPIDKp, gConfig.speedPIDKi, gConfig.speedPIDKd);
            speedPIDOutput = -speedPID.compute(speedPIDInput);

            /* Compute Angle PID (input is current angle and output is PWM percentage) */
            /* Set angle setpoint and compensate to reach equilibrium point */
            anglePID.setSetpoint(speedPIDOutput+gConfig.calibratedZeroAngle);
            anglePID.setTunings(gConfig.anglePIDConKp, gConfig.anglePIDConKi, gConfig.anglePIDConKd);
            anglePIDOutput = anglePID.compute(anglePIDInput);

            /* Compute Heading PID (input is WHEEL_RADIUS*2/WHEEL_DISTANCE0.6 * 360/TICKS_PER_TURN= , velocity * 11/20=0.55 * 360/464=0.77) */
            headingPIDInput = (velocity1-velocity2)/2 * 0.42;
            headingPID.setSetpoint(gConfig.steering);
            headingPID.setTunings(gConfig.headingPIDKp, gConfig.headingPIDKi, gConfig.headingPIDKd);
            headingPIDOutput = headingPID.compute(headingPIDInput);

            /* Set PWM value */
            if (gConfig.started &&
                (abs(anglePIDInput) > (abs(gConfig.calibratedZeroAngle)-gConfig.anglePIDLowerLimit) && 
                abs(anglePIDInput) < (abs(gConfig.calibratedZeroAngle)+gConfig.anglePIDLowerLimit)))
            {
                motor1.setSpeedPercentage(anglePIDOutput+headingPIDOutput);
                motor2.setSpeedPercentage(anglePIDOutput-headingPIDOutput);
            } else 
            {
                motor1.motorOff();
                motor2.motorOff();
            }
                  
            /* notify event */
            sprintf(sendBuffer, "%d,%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f#", timestamp, dt, distance1, distance2, speedPIDInput, speedPIDOutput, anglePIDInput, anglePIDOutput, headingPIDInput, headingPIDOutput, gConfig.calibratedZeroAngle, gConfig.direction, gConfig.steering);
            xQueueSend(gQueueReply, &sendBuffer, (TickType_t) 0);
        }
        vTaskDelayUntil(&xLastWakeTime, DATA_INTERVAL/portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}