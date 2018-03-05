/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "pid/PID.h"

void control(void *pvParameter);

#endif