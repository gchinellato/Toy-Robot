/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: UDP Server
 * Owner: gchinellato
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "udp_server.h"
#include "../../main.h"

char ssid[] = "TP-LINK_533A6C"; //  your network SSID (name)
char pass[] = "00533A6C";    // your network password (use for WPA, or use as key for WEP)
unsigned int localPort = 5001;      // local port to listen on

WiFiUDP UdpServer;

void WifiInit(){
    WiFi.disconnect(true);
    
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Connected to wifi");
    printWifiStatus();

    Serial.println("\nStarting connection to server...");
    UdpServer.begin(localPort);
}

void udpServer(void *pvParameter){
    char packetBuffer[255];

    WifiInit();

    vTaskDelay(50);

    for(;;){
        /* if there's data available, read a packet */
        int packetSize = UdpServer.parsePacket();
        if (packetSize) {
            //Serial.print("Received packet of size ");
            //Serial.println(packetSize);

            /* read the packet into packetBuffer */
            int len = UdpServer.read(packetBuffer, 255);
            if (len > 0) {
                packetBuffer[len] = 0;
            }
            //Serial.println(packetBuffer);
            parseEvent(packetBuffer);
        }
        vTaskDelay(50);
    }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("signal strength (RSSI):");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void parseEvent(char *buffer){
    int command = 0;
    int size; 
    char *ret;

    /* split string into tokens */
    ret = strtok(buffer, ",");

    /* get command */
    command = atoi(ret);

    //get number of parameters
    //size = int(strtok(NULL, ","));

    switch (command)
    {
        case STARTED:
            gConfig.started = atoi(strtok(NULL, ","));
            break;
        case DIRECTION:
            gConfig.direction = atof(strtok(NULL, ","));
            gConfig.direction /= 20;
            break;
        case STEERING:
            gConfig.steering = atof(strtok(NULL, ","));
            gConfig.steering /= 20;
            break;
        case SPEED_PID:
            gConfig.speedPIDKp = atof(strtok(NULL, ","));
            gConfig.speedPIDKi = atof(strtok(NULL, ","));
            gConfig.speedPIDKd = atof(strtok(NULL, ","));
            break;                        
        case ANGLE_PID_CONS:
            gConfig.anglePIDConKp = atof(strtok(NULL, ","));
            gConfig.anglePIDConKi = atof(strtok(NULL, ","));
            gConfig.anglePIDConKd = atof(strtok(NULL, ","));
            gConfig.calibratedZeroAngle = atof(strtok(NULL, ","));
            gConfig.anglePIDLowerLimit = atof(strtok(NULL, ","));
            break;
        case ZERO_ANGLE:
            gConfig.calibratedZeroAngle = atof(strtok(NULL, ","));
            break;
        case ANGLE_LIMITE:
            gConfig.anglePIDLowerLimit = atof(strtok(NULL, ","));
            break;
        default:
            break;
    }
}