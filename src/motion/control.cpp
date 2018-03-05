/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "control.h"

void control(void *pvParameter)
{ 
  PID speedPID;
  PID anglePID;
  
  for(;;)
  {
    //getOrientation();

    //getSpeed();

    //speedPID.setTunings(config.speedPIDKp, config.speedPIDKi, config.speedPIDKd);
    //speedPIDOutput = speedPID.compute(speedPIDInput);
    //anglePIDOutput = anglePID.compute(anglePIDInput);

    //motor1.setSpeedPercentage();
    //motor2.setSpeedPercentage();

    //notify();
    vTaskDelay(10 / portTICK_RATE_MS); 
  }
  vTaskDelete(NULL);
}