/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include <driver/dac.h>
#include "control.h"
#include "pinmux/pinmux.h"
#include "imu/imu.h"
#include "imu/MPU9250.h"
#include "pid/PID.h"
#include "motion/motor/motor.h"
#include "motion/encoder/encoder.h"

//PID objects
PID speedPID;
PID anglePID;

//Motor objects
Motor motor1(DAC_CHANNEL_1);
Motor motor2(DAC_CHANNEL_2);

//Encoder objects
Encoder encoder1;
Encoder encoder2;

/* interrupt functions for counting revolutions in the encoders */
/* when the callback function is called due an interrup event on pinEncoderAx
 * and pinEncoderBx=true, then is clockwise, if not it is counter-clockwise
 */
portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;

void encoderISR1()
{
    portENTER_CRITICAL(&mux1);
    if(digitalRead(ENCODERB1_PIN))
    { encoder1.ticks++; }
    else
    { encoder1.ticks--; }
    portEXIT_CRITICAL(&mux1);
}

void encoderISR2()
{
    portENTER_CRITICAL(&mux2);
    if(digitalRead(ENCODERB2_PIN))
    { encoder2.ticks++; }
    else
    { encoder2.ticks--; }
    portEXIT_CRITICAL(&mux2);
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
  vTaskDelay(50);
  float dt=0;
  unsigned long timestamp=0;
  unsigned long timestamp_old=0;
  float *ori;
  float speedPIDInput, anglePIDInput;
  float speedPIDOutput, anglePIDOutput;
  boolean started = true;
  //GY80 imu;
  MPU9250 myIMU;
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  myIMU.initMPU9250();

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

      //getEvent()

      //ori = imu.getOrientation(1, dt);
      ori = myIMU.getOrientation(1, dt);
      Serial.println("dt: " + String(dt) + ", Roll: " + String(ori[0]) + ", Pitch: " + String(ori[1]) + ", Yaw: " + String(ori[2]));
      anglePIDInput = ori[0]; //ori[1];

      //getSpeed();
      motor1.motorSpeed = (float)(encoder1.ticks - encoder1.lastTicks);
      encoder1.lastTicks = encoder1.ticks;
      motor2.motorSpeed = (float)(encoder2.ticks - encoder2.lastTicks);
      encoder2.lastTicks = encoder2.ticks;

      //Set angle setpoint and compensate to reach equilibrium point
      anglePID.setSetpoint(userControl.direction+configuration.calibratedZeroAngle);
      anglePID.setTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);
      //Compute Angle PID (input is current angle)
      anglePIDOutput = anglePID.compute(anglePIDInput);
      Serial.println("anglePIDoutput: " + String(anglePIDOutput));

      //Set PWM value
      if (started && (abs(anglePIDInput) < ANGLE_IRRECOVERABLE)) {
          motor1.setSpeedPercentage(anglePIDOutput+userControl.steering);
          motor2.setSpeedPercentage(anglePIDOutput-userControl.steering);
      }
      else {
          motor1.motorOff();
          motor2.motorOff();
      }

      //notify();
    }
    vTaskDelay(DATA_INTERVAL / portTICK_RATE_MS); 
  }
  vTaskDelete(NULL);
}