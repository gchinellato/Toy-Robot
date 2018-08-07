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
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "pinmux/pinmux.h"
#include "motion/control.h"
#include "motion/pid/PID.h"
#include "motion/encoder/encoder.h"
#include "comm/UDP/udp_server.h"
#include "comm/UDP/udp_client.h"

//inter-task communication queue
char buffer[255];
QueueHandle_t gQueueEvent = xQueueCreate(1, sizeof(buffer));
QueueHandle_t gQueueReply = xQueueCreate(1, sizeof(buffer));

/**
  * @brief Main App Entry point
  */
void setup()
{
  //Serial Init
  Serial.begin(115200);
  Serial.setTimeout(10);
  while(!Serial) {}

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

  if(gQueueEvent == NULL || gQueueReply == NULL){
    Serial.println("Error creating the queue");
  }

  delay(1000);

  xTaskCreate(&control, "control", configMINIMAL_STACK_SIZE+8192, NULL, 255, NULL);
  //xTaskCreate(&udpServer, "UDP_Server", configMINIMAL_STACK_SIZE+8192, NULL, 10, NULL);
  //xTaskCreate(&udpClient, "UDP_Client", configMINIMAL_STACK_SIZE+8192, NULL, 12, NULL);
}

void loop()
{
  //interface
  vTaskDelay(1000);
}
