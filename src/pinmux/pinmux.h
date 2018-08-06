/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: GPIO mapping
 * Owner: gchinellato
 */

#ifndef PINMUX_H
#define PINMUX_H

#define I2C_SDA     GPIO_NUM_18
#define I2C_SCL     GPIO_NUM_19
#define I2C_FREQ    100000

#define PWM1_PIN GPIO_NUM_5
#define CW1_PIN GPIO_NUM_17
#define CCW1_PIN GPIO_NUM_16
#define CS1_PIN 2

#define PWM2_PIN GPIO_NUM_4
#define CW2_PIN GPIO_NUM_2
#define CCW2_PIN GPIO_NUM_15
#define CS2_PIN 3

#define ENCODERA1_PIN GPIO_NUM_23
#define ENCODERB1_PIN GPIO_NUM_22
#define ENCODERA2_PIN GPIO_NUM_1
#define ENCODERB2_PIN GPIO_NUM_3

#endif