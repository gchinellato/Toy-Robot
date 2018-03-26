/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#include <stdio.h>
#include <string.h>
#include "control.h"
#include "imu/imu.h"

void control(void *pvParameter)
{ 
  vTaskDelay(50);
  float dt=0; // duration time
  unsigned long timestamp=0;
  unsigned long timestamp_old=0;
  float *ori; // orientation vector (roll, pitch, yaw)
  GY80 imu;
  vTaskDelay(50);

  for(;;)
  {
    timestamp = millis();
    if ((timestamp - timestamp_old) >= DATA_INTERVAL)
    {
      //convert from ms to sec
      dt = (float)(timestamp - timestamp_old)/1000.0f; 
      timestamp_old = timestamp;

      ori = imu.getOrientation(1, dt);
      Serial.println("dt: " + String(dt) + ", Roll: " + String(ori[0]) + ", Pitch: " + String(ori[1]) + ", Yaw: " + String(ori[2]));

      //getSpeed();

      //speedPID.setTunings(config.speedPIDKp, config.speedPIDKi, config.speedPIDKd);
      //speedPIDOutput = speedPID.compute(speedPIDInput);
      //anglePIDOutput = anglePID.compute(anglePIDInput);

      //motor1.setSpeedPercentage();
      //motor2.setSpeedPercentage();

      //notify();
    }
    vTaskDelay(DATA_INTERVAL / portTICK_RATE_MS); 
  }
  vTaskDelete(NULL);
}