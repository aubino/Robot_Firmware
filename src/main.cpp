#include <Arduino.h>
#include "Rotary.h"
#include "encoders.h"
#include "esp_timer.h"
#include <ESP32TimerInterrupt.h>
#include "freertos/task.h"
#include "servoing.h"

#define PWM 16
#define TIMER0_INTERVAL_MS        1
#define DEBOUNCING_INTERVAL_MS    80

/// --------- Here we will define our global variables -----------------------------///

Wheel left_wheel , right_wheel ; /*, right_wheel(0,1,2,3,4,5)*/
Wheel* left_wheel_ptr = &left_wheel ; 
Wheel* right_wheel_ptr = &right_wheel ; 
double left_wheel_error_sum = 0 ; 
double left_wheel_command = 0 ; 
double left_wheel_past_speed = 0 ; 


void IRAM_ATTR on_change_left() { onWheelInterrupt(&left_wheel) ;}

void IRAM_ATTR on_change_right() {onWheelInterrupt(&right_wheel) ;}

void IRAM_ATTR update_wheels_buffers() { updateWheelBuffers(&left_wheel) ; updateWheelBuffers(&right_wheel); }

bool IRAM_ATTR get_command(void* timerNo)
{
  left_wheel_command = proportinnal_command(3.14,
                          left_wheel_past_speed,
                          left_wheel.speed, 
                          &left_wheel_error_sum) ;
  left_wheel_past_speed = left_wheel.speed ; 
  return true ; 
}

void speedUpdater(Wheel * wheel_ptr)
{ 
  for(;;) //ininite loop
  {
    updateWheelSpeed(wheel_ptr);
    vTaskDelay(1/(DEFAULT_WHEEL_COMMAND_FREQUENCY*portTICK_PERIOD_MS)) ; 
  }
}


void  report_wheel_pose(Wheel* wheel_ptr)
{
  Serial.write("Report of the pose \n") ;
  Serial.print(wheel_ptr->speed) ; 
  Serial.write("\n");
  Serial.write("Report of the speed \n") ;
  Serial.print(wheel_ptr->speed) ; 
  Serial.write("\n");
  Serial.write("Report of the buffers \n");
  for(int i = 0; i<wheel_ptr->position_buffer.buffer_size ; i++)
  {
    Serial.print(wheel_ptr->position_buffer.internal_buffer[i]) ; 
    Serial.write("\t");
  }
  Serial.write("\n");
  for(int i = 0; i<wheel_ptr->timer_buffer.buffer_size ; i++)
  {
    Serial.print(wheel_ptr->timer_buffer.internal_buffer[i]) ; 
    Serial.write("\t");
  }  
  Serial.write("\n");
  Serial.print("Current index ") ; 
  Serial.print(wheel_ptr->position_buffer.current_index) ; 
  Serial.write("\n") ;
  Serial.write("End of reports \n");
}

static const uint16_t timer_divider = 80 ; 
static const uint64_t buffer_updating_timer_max_count = 1000000  / (DEFAULT_WHEEL_COMMAND_FREQUENCY / 2); 
static hw_timer_t* buffer_updating_timer = NULL ;  

void setup() {
  initWheel(left_wheel_ptr, 17,16,15,2,4,4,30) ; 
  Serial.begin(9600) ;
  setup_wheel(&left_wheel) ;
  setup_wheel(&right_wheel) ; 
  attachInterrupt(left_wheel.pina,on_change_left,CHANGE) ;
  attachInterrupt(left_wheel.pinb,on_change_left,CHANGE) ;
  attachInterrupt(right_wheel.pina,on_change_right,CHANGE);
  attachInterrupt(right_wheel.pinb,on_change_right,CHANGE) ; 
  // create and start the buffer push timer 
  buffer_updating_timer = timerBegin(0,timer_divider,true) ; 
  // attach ISR on timer 
  timerAttachInterrupt(buffer_updating_timer,&update_wheels_buffers,true) ; 
  // Set trigger time of the TIMER counter
  timerAlarmWrite(buffer_updating_timer,buffer_updating_timer_max_count,true) ; 
  // Now since i perform floating point arithmetics in the wheel speed calculation, it might be better to use freeRTOS tasks to 
  // make those calculations in parallel. at a certain frequency rather than calling them in ISR.
  
  timerAlarmEnable(buffer_updating_timer) ; 
}

void loop() {
  report_wheel_pose(&left_wheel);
  //left_wheel._update_speed() ; 
  delay(500) ; 
}

