#include <Arduino.h>
#include "Rotary.h"
#include "encoders.h"
#include "esp_timer.h"
#include <ESP32TimerInterrupt.h>

#define PWM 16
#define TIMER0_INTERVAL_MS        1
#define DEBOUNCING_INTERVAL_MS    80

Wheel left_wheel(18,19,17,5,16,16,30) ;/*, right_wheel(0,1,2,3,4,5)*/ 

auto led_state = HIGH ; 

void IRAM_ATTR on_change_left() { left_wheel._on_change() ;}

bool IRAM_ATTR update_left_wheel_buffer(void * timerNo) { left_wheel._update_buffers() ; return true ; } 

bool IRAM_ATTR update_left_wheel_speed(void * timerNo) { left_wheel._update_speed() ; return true ; } 

void  report_wheel_pose(Wheel* wheel_ptr)
{
  Serial.write("Report of the pose \n") ;
  Serial.print(wheel_ptr->getPosition()) ; 
  Serial.write("\n");
  Serial.write("Report of the speed \n") ;
  Serial.print(wheel_ptr->getSpeed()) ; 
  Serial.write("\n");
  Serial.write("Report of the buffers \n");
  for(int i = 0; i<wheel_ptr->position_buffer.buffer_size ; i++)
  {
    Serial.print(wheel_ptr->position_buffer.internal_buffer[i]) ; 
    Serial.write("\t");
  }
  Serial.write("\n");
  Serial.write("End of reports \n");
}

hw_timer_t* speed_sampler_timer = NULL;

ESP32Timer left_speed_sampler(0);

void setup() {
  Serial.begin(115200) ;
  left_wheel.setup() ;
  pinMode(LED_BUILTIN,OUTPUT);
  // left_wheel.applyVoltage(3.0) ; 
  attachInterrupt(left_wheel.getPinA(),on_change_left,CHANGE) ;
  attachInterrupt(left_wheel.getPinB(),on_change_left,CHANGE) ;
  if (left_speed_sampler.attachInterruptInterval(TIMER0_INTERVAL_MS*1000000/DEFAULT_WHEEL_COMMAND_FREQUENCY,update_left_wheel_buffer))
  {
    Serial.print(F("Left wheel sampling TImer "));
		Serial.println(millis());
	}
	else
		Serial.println(F("Can't set ITimer0. Select another freq. or timer"));

	Serial.flush();

  // speed_sampler_timer = timerBegin(0,DEFAULT_WHEEL_COMMAND_FREQUENCY,true);
}

void loop() {
  left_wheel._update_speed() ; 
  report_wheel_pose(&left_wheel);
  delay(1000) ; 
}

