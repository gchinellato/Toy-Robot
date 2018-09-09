/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include "../../motion/control.h"

void udpServer(void *pvParameter);
void printWifiStatus();
void WifiInit();
void parseEvent(char *buffer);

#endif