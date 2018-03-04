/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "driver/gpio.h"

#define BLINK_GPIO 2

QueueHandle_t queue;
SemaphoreHandle_t xBinarySemaphore;

typedef struct xMsg{
  uint32_t index;
  char name[10];
} xMsg;
  
void task1(void *pvParameter)
{ 
  xMsg msg;
  for(int i = 0; i<10; i++)
  {
    msg.index = i;
    sprintf(msg.name, "index:%d", i);
    xQueueSend(queue, &msg, portMAX_DELAY);
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

void task2(void *pvParameter)
{ 
  xMsg msg;

  for(int i = 0; i<10; i++)
  {
    xQueueReceive(queue, &msg, portMAX_DELAY);
    printf("%d %s.\n", msg.index, msg.name);
  }
  vTaskDelete(NULL);
}
 
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
  /* 
    - SystemControl thread
        Read IMU
        Sensor fusion
        Read Encoder
        Update speed
        Compute speed PID
        Compute angle PID
        Set motor speed 
    - DistanceMeasure thread
        Read IR sensor
        Send notification to event handler or set motor speed?
    - EventHandler thread
        Receive events from Raspberry or other threads
    - BluetoothTriangulation thread
    - Debug thead
  */

  queue = xQueueCreate(10, sizeof(xMsg));
  xBinarySemaphore = xSemaphoreCreateBinary();
  //xSemaphoreTake(xBinarySemaphore, portMAX_DELAY); //block waiting for event
  //xSemaphoreGive(xBinarySemaphore); //unblock, sempahore is free

  xTaskCreate(&task1, "producerTask", configMINIMAL_STACK_SIZE+1024, NULL, 2, NULL);
  xTaskCreate(&task2, "consumerTask", configMINIMAL_STACK_SIZE+1024, NULL, 1, NULL);
  xTaskCreate(&blink, "blink", 512,NULL,0,NULL);

  /* should never reach here! */ 
  for(;;);
}
