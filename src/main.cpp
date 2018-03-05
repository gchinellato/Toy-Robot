/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Entry point
 * Owner: gchinellato
 */

extern "C" {
   void app_main();
}

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "motion/control.h"

#define BLINK_GPIO GPIO_NUM_2

void blink(void *pvParameter)
{ 
  gpio_pad_select_gpio(BLINK_GPIO);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

  for(;;)
  {
    /* Blink off (output low) */
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(1000 / portTICK_RATE_MS);
    /* Blink on (output high) */
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

/**
  * @brief Main App Entry point
  */
void app_main()
{
  xTaskCreate(&control, "control", configMINIMAL_STACK_SIZE+1024, NULL, 2, NULL);
  //xTaskCreate(&eventHandler, "eventHandler", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL);
  //xTaskCreate(&detection, "detection", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL);
  //xTaskCreate(&triangulation, "triangulation", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL);
  xTaskCreate(&blink, "blink", 512,NULL,0,NULL);

  /* should never reach here! */ 
  for(;;);
}
