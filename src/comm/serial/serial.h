/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Serial
 * Owner: gchinellato
 */

#ifndef SERIAL_H
#define SERIAL_H

#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 5
#define BUFFER_MAX 128

void serial(void *pvParameter);
void parse(char *buffer);

#endif