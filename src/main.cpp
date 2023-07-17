#include <Arduino.h>
#include "Rotary.h"
#include "encoders.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "servoing.h"
#include "data_transfert.h"
#include <geometry_msgs/Vector3Stamped.h> 
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

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
double right_wheel_error_sum = 0 ; 
double right_wheel_command = 0 ; 
double right_wheel_past_speed = 0 ; 
// ------------------------- Ros Parameters Declarations ---------------------------------------//
geometry_msgs::Vector3Stamped left_wheel_data,right_wheel_data ;
std_msgs::Float64 left_wheel_ros_command , right_wheel_ros_command ; 
ros::Publisher left_wheel_data_publisher("/left_wheel/command/get",&left_wheel_data) ; 
ros::Publisher right_wheel_data_publisher("/right_wheel/command/get",&right_wheel_data) ;

void getLeftWheelCommand( const std_msgs::Float64& left_wheel_command_recieved)
{
  left_wheel_ros_command.data = left_wheel_command_recieved.data ; 
}

void getRightWheelCommand( const std_msgs::Float64& right_wheel_command_recieved)
{
  right_wheel_ros_command.data = right_wheel_command_recieved.data ; 
}

ros::Subscriber<std_msgs::Float64> left_wheel_command_sub("/left_wheel/command/set", &getLeftWheelCommand ); 
ros::Subscriber<std_msgs::Float64> right_wheel_command_sub("/right_wheel/command/set", &getRightWheelCommand); 


void IRAM_ATTR on_change_left() { onWheelInterrupt(&left_wheel) ;}

void IRAM_ATTR on_change_right() {onWheelInterrupt(&right_wheel) ;}

void IRAM_ATTR update_wheels_buffers() { updateWheelBuffers(&left_wheel) ; updateWheelBuffers(&right_wheel); }

void wheelCommandingTimerCallback(TimerHandle_t xTimer)
{
  left_wheel_command = pid_command(left_wheel_ros_command.data,
                          left_wheel_past_speed,
                          left_wheel.speed, 
                          &left_wheel_error_sum) ;
  right_wheel_command = pid_command(right_wheel_ros_command.data,
                          right_wheel_past_speed,
                          right_wheel.speed, 
                          &right_wheel_error_sum) ;
  left_wheel_past_speed = left_wheel_ptr->speed ; 
  right_wheel_past_speed = right_wheel_ptr->speed ; 
  applyVoltageToWheel(left_wheel_ptr,left_wheel_command) ; 
  applyVoltageToWheel(right_wheel_ptr,right_wheel_command) ; 
  Serial.print((String)"left_wheel_command : " + left_wheel_command +"\n") ;
  Serial.print((String)"right_wheel_command : " + right_wheel_command +"\n") ;
}

void bufferUpdatingTimerCallback(TimerHandle_t xTimer)
{
  portENTER_CRITICAL(&spinlock) ;
  updateWheelBuffers(left_wheel_ptr) ; 
  updateWheelBuffers(right_wheel_ptr);
  portEXIT_CRITICAL(&spinlock) ; 
}

void speedUpdatingTimerCallback(TimerHandle_t xTimer)
{
  updateWheelSpeed(left_wheel_ptr);
  updateWheelSpeed(right_wheel_ptr);
  left_wheel_data.header.frame_id = "left_wheel_frame"        ; right_wheel_data.header.frame_id = "right_wheel_frame"        ; 
  left_wheel_data.header.stamp = nh.now()                     ; right_wheel_data.header.stamp = nh.now()                      ; 
  left_wheel_data.vector.x = left_wheel_ptr->speed            ; right_wheel_data.vector.x = right_wheel_ptr->speed            ;
  left_wheel_data.vector.y = left_wheel_ptr->encoder_position ; right_wheel_data.vector.y = right_wheel_ptr->encoder_position ;
  left_wheel_data.vector.z = 0.0                              ; right_wheel_data.vector.z = 0.0                               ;
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

void wheel_reporting_task(void* wheel_ptr)
{
  Wheel* wheel_ptr_casted = (Wheel*) wheel_ptr ;
  for(;;) //infinite loop 
  {
    printWheelState(wheel_ptr_casted) ; 
    delay(1000) ; 
  }
}



TaskHandle_t leftspeedReportingtaskHandle = NULL;
static TimerHandle_t speedUpdatingTimer = NULL ;
static TimerHandle_t bufferUpdatingTimer = NULL ;  
static TimerHandle_t wheelCommandingTimer = NULL ; 



void setup() {
  setupWiFi();
  nh.initNode() ;
  nh.getHardware()->setConnection(server, serverPort);
  nh.advertise(left_wheel_data_publisher);
  nh.advertise(right_wheel_data_publisher);
  nh.subscribe(left_wheel_command_sub);
  nh.subscribe(right_wheel_command_sub);
  initWheel(left_wheel_ptr, 17,16,15,2,4,4,210) ;
  initWheel(right_wheel_ptr,12,13,27,26,14,14,210);  
  Serial.begin(250000) ;
  setup_wheel(&left_wheel) ;
  setup_wheel(&right_wheel) ; 
  attachInterrupt(left_wheel.pina,on_change_left,CHANGE) ;
  attachInterrupt(left_wheel.pinb,on_change_left,CHANGE) ;
  attachInterrupt(right_wheel.pina,on_change_right,CHANGE);
  attachInterrupt(right_wheel.pinb,on_change_right,CHANGE) ; 
  // xTaskCreatePinnedToCore(wheel_reporting_task,"LeftWheelPoseReporter",1024,(void *)&left_wheel,0,&leftspeedReportingtaskHandle,0) ; 
  // I plan on using the software timers of FreeRtos(Timer Daemon) because i'm essentially just doing that. and my task priorities are messes up.
  // And also for some obscure reason, IDLE task keeps triggering the watchdog because of the priorities.
  speedUpdatingTimer = xTimerCreate("speedUpdatingTimer", //name of the timer
                                    (2*1000)/(DEFAULT_WHEEL_COMMAND_FREQUENCY*portTICK_PERIOD_MS), //period of timer in ticks = 50HZ
                                    pdTRUE, // auto reload 
                                    (void*) 0, // timer index
                                    speedUpdatingTimerCallback // Callback function
                                    ) ; 
  bufferUpdatingTimer = xTimerCreate("bufferUpdatingTimer" , 
                                      1000/(DEFAULT_WHEEL_COMMAND_FREQUENCY*portTICK_PERIOD_MS) , //period of timer in ticks = 100HZ
                                      pdTRUE , 
                                      (void*) 1,
                                      bufferUpdatingTimerCallback) ; 
  wheelCommandingTimer = xTimerCreate("wheelCommandingTimer" , 
                                      (2*1000)/(DEFAULT_WHEEL_COMMAND_FREQUENCY*portTICK_PERIOD_MS) , //period of timer in ticks = 50HZ
                                      pdTRUE , 
                                      (void*) 1,
                                      wheelCommandingTimerCallback) ; 
  // Ok long story short, the speed is not updating despite my calculations. 
  //I figured something is going wrong in the updateWheelSpeed function in encoder.cpp file . 
  
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
  
  if(wheelCommandingTimer== NULL)
  {
    Serial.print("Clould not create a speed updating timer") ; 
    delay(5000) ; 
  }
  
  else
  {
    Serial.print("Succesfully created Wheel commanding Timer ") ; 
    xTimerStart(wheelCommandingTimer,portMAX_DELAY) ; 
  }
  
  // vTaskStartScheduler() ; 
  // applyVoltageToWheel(&left_wheel, 6.0); 
  // applyVoltageToWheel(&right_wheel,-6.0);
}

void loop() 
{
  left_wheel_data_publisher.publish(&left_wheel_data) ; 
  right_wheel_data_publisher.publish(&right_wheel_data) ; 
  nh.spinOnce();
  delay(uint32_t((2*1000)/DEFAULT_WHEEL_COMMAND_FREQUENCY)) ; 
}

