/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Entry point
 * Owner: gchinellato
 */

extern "C" {
   void app_main();
}

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "pinmux/pinmux.h"
#include "motion/control.h"
#include "motion/pid/PID.h"
#include "motion/encoder/encoder.h"

/**
  * @brief Main App Entry point
  */
void setup()
{
  //Serial Init
  Serial.begin(115200);
  Serial.setTimeout(10);
  while(!Serial) {}

  //I2C Init
  Wire.begin(I2C_SCL, I2C_SDA, I2C_FREQ);

  xTaskCreate(&control, "control", configMINIMAL_STACK_SIZE+1024, NULL, 10, NULL);
}

void loop()
{
  //interface
  vTaskDelay(1000);
}