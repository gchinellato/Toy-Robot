/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Serial 
 * Owner: gchinellato
 */

#include <Arduino.h>
#include "serial.h"
#include "../../motion/control.h"
#include "../../main.h"

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void serial(void *pvParameter)
{    	
    String msg;
	char buff[BUFFER_MAX];

    /* Serial Init */
	Serial.begin(BAUD_RATE);
	Serial.setTimeout(SERIAL_TIMEOUT);	
	while(!Serial) {}

	for(;;){
		if(Serial.available()){
			msg = Serial.readStringUntil('#');
            Serial.println(msg);
            if(msg.indexOf(',') != -1){
                msg.toCharArray(buff, BUFFER_MAX);		
			    parse(buff);
            }
		}
		
		if(xQueueReceive(gQueueReply, &buff, portMAX_DELAY)){
			Serial.println(buff);
		}	
	}
	vTaskDelete(NULL);
}

void parse(char *buffer){
    int command = 0; 
    char *ret;

    /* split string into tokens */
    ret = strtok(buffer, ",");

    /* get command */
    command = atoi(ret);

    taskENTER_CRITICAL(&mux);
    /* (module),(data1),(data2),(data3),(...)(#) */
    switch (command)
    {
        case STARTED:
            gConfig.started = atoi(strtok(NULL, ","));
            break;
        case DIRECTION:
            gConfig.direction = -atof(strtok(NULL, ","));
            break;
        case STEERING:
            gConfig.steering = atof(strtok(NULL, ","));
            break;
        case SPEED_PID:
            gConfig.speedPIDKp = atof(strtok(NULL, ","));
            gConfig.speedPIDKi = atof(strtok(NULL, ","));
            gConfig.speedPIDKd = atof(strtok(NULL, ","));
            break;                        
        case ANGLE_PID:
            gConfig.anglePIDConKp = atof(strtok(NULL, ","));
            gConfig.anglePIDConKi = atof(strtok(NULL, ","));
            gConfig.anglePIDConKd = atof(strtok(NULL, ","));
            break;
        case HEADING_PID:
            gConfig.headingPIDKp = atof(strtok(NULL, ",")) / 100;
            gConfig.headingPIDKi = atof(strtok(NULL, ",")) / 100;
            gConfig.headingPIDKd = atof(strtok(NULL, ",")) / 100;
            break;
        case ZERO_ANGLE:
            gConfig.calibratedZeroAngle = atof(strtok(NULL, ","));
            break;
        case ANGLE_LIMITE:
            gConfig.anglePIDLowerLimit = atof(strtok(NULL, ","));
            break;
        case CF_IMU:
            gConfig.cf = atof(strtok(NULL, ","));
            break;
        case RESET_ENCODER:
            encoder1.resetTicks();
            encoder2.resetTicks();
            break;
        case POS_CONTROL:
            gConfig.posControlFlag = atoi(strtok(NULL, ","));
            break;
        default:
            break;
    }
    taskEXIT_CRITICAL(&mux);
}