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
#include "imu/MPU9250.h"
#include "pid/PID.h"
#include "motion/motor/motor.h"
#include "motion/encoder/encoder.h"
#include "../main.h"

//PID objects
PID speedPID;
PID anglePID;

//Motor objects
Motor motor1(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM0A, PWM1_PIN, CW1_PIN, CCW1_PIN, CS1_PIN);
Motor motor2(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM0B, PWM2_PIN, CW2_PIN, CCW2_PIN, CS2_PIN);

//Encoder objects
Encoder encoder1;
Encoder encoder2;

/* interrupt functions for counting revolutions in the encoders */
/* when the callback function is called due an interrup event on pinEncoderAx
 * and pinEncoderBx=true, then is clockwise, if not it is counter-clockwise
 */
portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;

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

void setConfiguration(Configuration *configuration)
{
    configuration->speedPIDKp = SPEED_KP;
    configuration->speedPIDKi = SPEED_KI;
    configuration->speedPIDKd = SPEED_KD;
    configuration->speedPIDOutputLowerLimit = -10.00;
    configuration->speedPIDOutputHigherLimit = 10.00;
    configuration->anglePIDAggKp = ANGLE_KP_AGGR;
    configuration->anglePIDAggKi = ANGLE_KI_AGGR;
    configuration->anglePIDAggKd = ANGLE_KD_AGGR;
    configuration->anglePIDConKp = ANGLE_KP_CONS;
    configuration->anglePIDConKi = ANGLE_KI_CONS;
    configuration->anglePIDConKd = ANGLE_KD_CONS;
    configuration->anglePIDLowerLimit = ANGLE_LIMIT;
    configuration->calibratedZeroAngle = CALIBRATED_ZERO_ANGLE;
}

void control(void *pvParameter)
{
    //I2C Init  
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    //Interrupt Init
    pinMode(ENCODERA1_PIN, INPUT_PULLUP);
    pinMode(ENCODERA2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODERA1_PIN), encoderISR1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODERA2_PIN), encoderISR2, RISING);

    //PMM Init
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 5000;    //frequency = 5000Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    vTaskDelay(50);

    float dt=0;
    float velocity1=0, velocity2=0;
    float distance1, distance2;
    float lastDistance1 = 0, lastDistance2 = 0;
    unsigned long timestamp=0;
    unsigned long timestamp_old=0;
    float *ori;
    float speedPIDInput, anglePIDInput;
    float speedPIDOutput, anglePIDOutput;
    boolean started = false;
    GY80 imu;

    int command = 0;
    int size; 
    char *ret;
    char receivedBuffer[255];
    char sendBuffer[255];

    UserControl userControl = {0, 0};
    Configuration configuration;  

    setConfiguration(&configuration);

    vTaskDelay(50);

    for(;;)
    {
        timestamp = millis();
        if ((timestamp - timestamp_old) >= DATA_INTERVAL)
        {
            //convert from ms to sec
            dt = (float)(timestamp - timestamp_old)/1000.0f; 
            timestamp_old = timestamp;

            //getEvent
            if(uxQueueMessagesWaiting(gQueueEvent))
            {
                xQueueReceive(gQueueEvent, &receivedBuffer, portMAX_DELAY);

                //split string into tokens
                ret = strtok(receivedBuffer, ",");

                //get command
                command = atoi(ret);

                //get number of parameters
                //size = int(strtok(NULL, ","));

                switch (command)
                {
                    case STARTED:
                        started = atoi(strtok(NULL, ","));
                        //Serial.println("STARTED command: " + String(started));
                        break;
                    case DIRECTION:
                        userControl.direction = atof(strtok(NULL, ","));
                        userControl.direction /= 10;
                        //Serial.println("DIRECTION command: " + String(userControl.direction));
                        break;
                    case STEERING:
                        userControl.steering = atof(strtok(NULL, ","));
                        userControl.steering /= 10;
                        //Serial.println("STEERING command: " + String(userControl.steering));
                        break;
                    case ANGLE_PID_CONS:
                        configuration.anglePIDConKp = atof(strtok(NULL, ","));
                        configuration.anglePIDConKi = atof(strtok(NULL, ","));
                        configuration.anglePIDConKd = atof(strtok(NULL, ","));
                        configuration.calibratedZeroAngle = atof(strtok(NULL, ","));
                        configuration.anglePIDLowerLimit = atof(strtok(NULL, ","));
                        //Serial.println("ANGLE_PID_CONS command: " + String(configuration.anglePIDConKp) +","+ String(configuration.anglePIDConKi) +","+ String(configuration.anglePIDConKd));
                        //Serial.println("ZERO_ANGLE command: " + String(configuration.calibratedZeroAngle));
                        //Serial.println("ANGLE_LIMITE command: " + String(configuration.anglePIDLowerLimit));
                        break;
                    case ZERO_ANGLE:
                        configuration.calibratedZeroAngle = atof(strtok(NULL, ","));
                        //Serial.println("ZERO_ANGLE command: " + String(configuration.calibratedZeroAngle));
                        break;
                    case ANGLE_LIMITE:
                        configuration.anglePIDLowerLimit = atof(strtok(NULL, ","));
                        //Serial.println("ANGLE_LIMITE command: " + String(configuration.anglePIDLowerLimit));
                        break;
                    default:
                        //Serial.println("Unknown command");
                        break;
                }
            }

            ori = imu.getOrientation(1, dt);
            //Serial.println("dt: " + String(dt) + ", Roll: " + String(ori[0]) + ", Pitch: " + String(ori[1]) + ", Yaw: " + String(ori[2]));
            anglePIDInput = ori[1];

            //getSpeed
            distance1 = encoder1.getDistance();
            distance2 = encoder2.getDistance();
            velocity1 = distance1 - lastDistance1;
            velocity2 = distance2 - lastDistance2;
            lastDistance1 = distance1;
            lastDistance2 = distance2;
            //Serial.println("distance1: " + String(distance1) + ", velocity1: " + String(velocity1));

            // PID Speed
            speedPIDInput = velocity1;
            speedPID.setSetpoint(userControl.direction);
            speedPID.setTunings(configuration.speedPIDKp, configuration.speedPIDKi, configuration.speedPIDKd);
            //Compute Speed PID (input is wheel speed. output is angleSetpoint)
            speedPIDOutput = speedPID.compute(speedPIDInput);

            // PID Angle
            //Set angle setpoint and compensate to reach equilibrium point
            anglePID.setSetpoint(speedPIDOutput+configuration.calibratedZeroAngle);
            anglePID.setTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);
            //Compute Angle PID (input is current angle)
            anglePIDOutput = anglePID.compute(anglePIDInput);
            //Serial.println("anglePIDoutput: " + String(anglePIDOutput));

            //Set PWM value
            if (started &&
                (abs(anglePIDInput) > (abs(configuration.calibratedZeroAngle)-configuration.anglePIDLowerLimit) && 
                abs(anglePIDInput) < (abs(configuration.calibratedZeroAngle)+configuration.anglePIDLowerLimit)))
            {
                motor1.setSpeedPercentage(anglePIDOutput+userControl.steering);
                motor2.setSpeedPercentage(anglePIDOutput-userControl.steering);
            } else 
            {
                motor1.motorOff();
                motor2.motorOff();
            }
                  
            //notify
            sprintf(sendBuffer, "%d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f#", timestamp, distance1, speedPIDInput, speedPIDOutput, anglePIDInput, anglePIDOutput);
            xQueueSend(gQueueReply, &sendBuffer, 10);
            Serial.println(String(sendBuffer));
        }
        vTaskDelay(DATA_INTERVAL / portTICK_RATE_MS); 
    }
    vTaskDelete(NULL);
}