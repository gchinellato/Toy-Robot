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
#include "main.h"
#include "pinmux/pinmux.h"
#include "motion/control.h"
#include "comm/serial/serial.h"

/* inter-task communication queue */
char buffer[BUFFER_MAX];
QueueHandle_t gQueueReply = xQueueCreate(128, sizeof(buffer));

/**
  * @brief Main App Entry point
  */
void setup()
{
  if(gQueueReply == NULL){
    Serial.println("Error creating the queue");
  }

  vTaskDelay(1000);

  xTaskCreate(&control, "control", configMINIMAL_STACK_SIZE+8192, NULL, 11, NULL);
  xTaskCreate(&serial, "serial", configMINIMAL_STACK_SIZE+8192, NULL, 10, NULL);
}

void loop()
{
  vTaskDelay(1000);
}
