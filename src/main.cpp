#include <Arduino.h>
#include "Rotary.h"
#include "encoders.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/timers.h"
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

void bufferUpdatingTimerCallback(TimerHandle_t xTimer)
{
  updateWheelBuffers(left_wheel_ptr) ; 
  updateWheelBuffers(right_wheel_ptr);
}

void speedUpdatingTimerCallback(TimerHandle_t xTimer)
{
  updateWheelSpeed(left_wheel_ptr);
  updateWheelSpeed(right_wheel_ptr);
}

void speedUpdater(void * wheel_ptr)
{ 
  Wheel* wheel_ptr_casted = (Wheel*) wheel_ptr ; 
  constexpr double ms = 1000/DEFAULT_WHEEL_COMMAND_FREQUENCY ;  
  for(;;) //ininite loop
  {
    updateWheelSpeed(wheel_ptr_casted);
    vTaskDelay(1/(ms*portTICK_PERIOD_MS)) ; 
  }
}


  /* The `report_wheel_pose` function is used to print out the current pose and speed of a wheel.
It takes a pointer to a `Wheel` object as an argument and prints out the position and timer
buffers, as well as the current speed of the wheel. This function is useful for debugging and
monitoring the behavior of the wheel. */
void report_wheel_pose(void* wheel_ptr)
{
  Wheel* wheel_ptr_casted = (Wheel*) wheel_ptr ;
  for(;;) //infinite loop 
  {
    Serial.write("Report of the pose \n") ;
    Serial.print(wheel_ptr_casted->encoder_position) ; 
    Serial.write("\n");
    Serial.write("Report of the speed \n") ;
    Serial.print(wheel_ptr_casted->speed) ; 
    Serial.write("\n");
    Serial.write("Report of the buffers \n");
    for(int i = 0; i<wheel_ptr_casted->position_buffer.buffer_size ; i++)
    {
      Serial.print(wheel_ptr_casted->position_buffer.internal_buffer[i]) ; 
      Serial.write("\t");
    }
    Serial.write("\n");
    for(int i = 0; i<wheel_ptr_casted->timer_buffer.buffer_size ; i++)
    {
      Serial.print(wheel_ptr_casted->timer_buffer.internal_buffer[i]) ; 
      Serial.write("\t");
    }  
    Serial.write("\n");
    Serial.print("Current index ") ; 
    Serial.print(wheel_ptr_casted->position_buffer.current_index) ; 
    Serial.write("\n") ;
    Serial.write("End of reports \n");
    delay(1000) ; 
  }
}

// TaskHandle_t leftspeedUpdatingtaskHandle = NULL;
// TaskHandle_t rightspeedUpdatingtaskHandle = NULL;
TaskHandle_t leftspeedReportingtaskHandle = NULL;
static TimerHandle_t speedUpdatingTimer = NULL ;
static TimerHandle_t bufferUpdatingTimer = NULL ;  



void setup() {
  initWheel(left_wheel_ptr, 17,16,15,2,4,4,30) ;
  initWheel(right_wheel_ptr,13,12,27,26,14,14,30);  
  Serial.begin(115200) ;
  setup_wheel(&left_wheel) ;
  setup_wheel(&right_wheel) ; 
  attachInterrupt(left_wheel.pina,on_change_left,CHANGE) ;
  attachInterrupt(left_wheel.pinb,on_change_left,CHANGE) ;
  attachInterrupt(right_wheel.pina,on_change_right,CHANGE);
  attachInterrupt(right_wheel.pinb,on_change_right,CHANGE) ; 
  // Now since i perform floating point arithmetics in the wheel speed calculation, it might be better to use freeRTOS tasks to 
  // make those calculations in parallel. at a certain frequency rather than calling them in ISR.
  // xTaskCreatePinnedToCore(speedUpdater,"LeftWheelSpeedUpdater",1024,(void * )&left_wheel,0,&leftspeedUpdatingtaskHandle,0) ; 
  // xTaskCreatePinnedToCore(speedUpdater,"RightWheelSpeedUpdater",1024,(void * )&right_wheel,0,&rightspeedUpdatingtaskHandle,0) ;
  xTaskCreatePinnedToCore(report_wheel_pose,"LeftWheelPoseReporter",1024,(void *)&left_wheel,0,&leftspeedReportingtaskHandle,0) ; 
  // I plan on using the software timers of FreeRtos(Timer Daemon) because i'm essentially just doing that. and my task priorities are messes up.
  // And also for some obscure reason, IDLE task keeps triggering the watchdog because of the priorities.
  speedUpdatingTimer = xTimerCreate("speedUpdatingTimer", //name of the timer
                                    2000/(DEFAULT_WHEEL_COMMAND_FREQUENCY*portTICK_PERIOD_MS), //period of timer in ticks
                                    pdTRUE, // auto reload 
                                    (void*) 0, // timer index
                                    speedUpdatingTimerCallback // Callback function
                                    ) ; 
  bufferUpdatingTimer = xTimerCreate("bufferUpdatingTimer" , 
                                      1000/(DEFAULT_WHEEL_COMMAND_FREQUENCY*portTICK_PERIOD_MS) , 
                                      pdTRUE , 
                                      (void*) 1,
                                      bufferUpdatingTimerCallback) ; 
  
  if(speedUpdatingTimer== NULL)
  {
    Serial.print("Clould not create a speed updating timer") ; 
    delay(5000) ; 
  }
  
  else
  {
    Serial.print("Succesfully created speed updating timer") ; 
    xTimerStart(speedUpdatingTimer,portMAX_DELAY) ; 
  }
  
  if(bufferUpdatingTimer== NULL)
  {
    Serial.print("Clould not create a buffer updating timer") ; 
    delay(5000) ; 
  }
  
  else
  {
    Serial.print("Succesfully created buffer updating timer") ; 
    xTimerStart(bufferUpdatingTimer,portMAX_DELAY) ; 
  }
  
  // vTaskStartScheduler() ; 
  applyVoltageToWheel(&left_wheel,4.0); 
  applyVoltageToWheel(&right_wheel,4.0);
}

void loop() {
  //Serial.print("Loop is running ") ; 
  //delay(1000); 
}

