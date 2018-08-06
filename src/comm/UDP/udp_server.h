/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#ifndef UDP_SERVER_H
#define UDP_SERVER_H

void udpServer(void *pvParameter);
void printWifiStatus();
void WifiInit();

#endif