/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include "Rotary.h"
#include "ressources.h"

#define LED 2
hw_timer_t* speed_sampler_timer = NULL;

void setup() {
  left_wheel.setup() ;
  attachInterrupt(left_wheel.getPinA(),on_change_left,CHANGE);
  speed_sampler_timer = timerBegin(0,DEFAULT_WHEEL_COMMAND_FREQUENCY,true);
  timerAttachInterrupt(speed_sampler_timer, &update_left_wheel_speed,true) ;

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH);
  Serial.println("LED is on");
  delay(1000);
  digitalWrite(LED, LOW);
  Serial.println("LED is off");
  delay(1000);
}

