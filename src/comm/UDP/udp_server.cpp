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

int status = WL_IDLE_STATUS;
char ssid[] = "TP-LINK_533A6C"; //  your network SSID (name)
char pass[] = "00533A6C";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 5001;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

void WifiInit(){
    WiFi.disconnect(true);
    
    Serial.println();
    Serial.println();
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
    // if you get a connection, report back via serial:
    Udp.begin(localPort);
}

void udpServer(void *pvParameter){
    WifiInit();

    vTaskDelay(50);

    printWifiStatus();

    for(;;){
        // if there's data available, read a packet
        int packetSize = Udp.parsePacket();
        if (packetSize) {
            Serial.print("Received packet of size ");
            Serial.println(packetSize);
            Serial.print("From ");
            IPAddress remoteIp = Udp.remoteIP();
            Serial.print(remoteIp);
            Serial.print(", port ");
            Serial.println(Udp.remotePort());

            // read the packet into packetBufffer
            int len = Udp.read(packetBuffer, 255);
            if (len > 0) {
                packetBuffer[len] = 0;
            }
            //Serial.print("Contents:");
            //Serial.println(packetBuffer);

            xQueueSend(gQueueEvent, &packetBuffer, portMAX_DELAY);
        }
        vTaskDelay(250);
    }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}