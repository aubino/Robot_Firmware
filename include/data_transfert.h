#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <ros/time.h>
#ifndef DATA_TRANSFERT_H
#define DATA_TRANSFERT_H
#define ROSSERIAL_ARDUINO_TCP
IPAddress server(10, 42, 0, 1);
uint16_t serverPort = 11411 ;
const char*  wifi_ssid = "aubino_wifi" ;
const char*  wifi_password = "X4v1pdCP" ;
ros::NodeHandle  nh ;

void setupWiFi()
{  
   WiFi.begin(wifi_ssid, wifi_password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("WIFI SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP ADRESS:   ");
   Serial.println(WiFi.localIP());

}
#endif