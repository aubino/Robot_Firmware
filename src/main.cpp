#include <Arduino.h>
#include "Rotary.h"
#include "encoders.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "servoing.h"
#include "data_transfert.h"
#include <geometry_msgs/Vector3Stamped.h> 
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include "odometry.h"
#define TIMER0_INTERVAL_MS        1
#define DEBOUNCING_INTERVAL_MS    80
#define SPEED_UPDATING_FREQUENCY  DEFAULT_WHEEL_COMMAND_FREQUENCY/2
#define BUFFER_UPDATING_FREQUENCY DEFAULT_WHEEL_COMMAND_FREQUENCY
#define ROS_PUBLISHING_FREQUENCY  DEFAULT_WHEEL_COMMAND_FREQUENCY/2

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
nav_msgs::Odometry odometry ; 
geometry_msgs::Pose2D robot_pose ; 
ros::Publisher left_wheel_data_publisher("/left_wheel/command/get",&left_wheel_data) ; 
ros::Publisher right_wheel_data_publisher("/right_wheel/command/get",&right_wheel_data) ;
ros::Publisher odometry_publisher("/odom",&odometry) ; 
ros::Publisher robot_pose_publisher("/robot/pose",&robot_pose) ; 

void getLeftWheelCommand( const std_msgs::Float64& left_wheel_command_recieved)
{
  left_wheel_ros_command.data = left_wheel_command_recieved.data ; 
}

void getRightWheelCommand( const std_msgs::Float64& right_wheel_command_recieved)
{
  right_wheel_ros_command.data = right_wheel_command_recieved.data ; 
}

void modifyPIDParameters(const geometry_msgs::Vector3& PID_parameters)
{
  double PID_P = PID_parameters.x ; 
  double PID_I = PID_parameters.y ; 
  double PID_D = PID_parameters.z ; 
  setPIDStateCoefficients(&(right_wheel_ptr->_internal_pid_state),PID_P, PID_I , PID_D , 1/DEFAULT_WHEEL_COMMAND_FREQUENCY) ; 
  setPIDStateCoefficients(&(left_wheel_ptr->_internal_pid_state),PID_P, PID_I , PID_D , 1/DEFAULT_WHEEL_COMMAND_FREQUENCY)  ;  
} 

void setLinearSpeed(const geometry_msgs::Twist& linear_speed)
{
  double speed_to_apply_to_left_wheel(0) , speed_to_apply_to_right(0) ; 
  computeInvertKinematics(robot_pose.theta,linear_speed.linear.x,linear_speed.linear.y,linear_speed.angular.z,speed_to_apply_to_left_wheel,speed_to_apply_to_right) ; 
  left_wheel_ros_command.data = speed_to_apply_to_left_wheel ; 
  right_wheel_ros_command.data = speed_to_apply_to_right ; 
  return ; 
}

ros::Subscriber<std_msgs::Float64> left_wheel_command_sub("/left_wheel/command/set", &getLeftWheelCommand ); 
ros::Subscriber<std_msgs::Float64> right_wheel_command_sub("/right_wheel/command/set", &getRightWheelCommand);
ros::Subscriber<geometry_msgs::Vector3> pid_param_modification_sub("/PID/set", &modifyPIDParameters);
// ros::Subscriber<geometry_msgs::Twist> robot_driving_sub("/robot/cmd_vel",&setLinearSpeed) ; 



void IRAM_ATTR on_change_left() { onWheelInterrupt(&left_wheel) ;}

void IRAM_ATTR on_change_right() {onWheelInterrupt(&right_wheel) ;}

void IRAM_ATTR update_wheels_buffers() { updateWheelBuffers(&left_wheel) ; updateWheelBuffers(&right_wheel); }

void wheelCommandingTimerCallback(TimerHandle_t xTimer)
{
  updatePID(&(left_wheel_ptr->_internal_pid_state) , left_wheel_ros_command.data, left_wheel_ptr->speed) ; 
  updatePID(&(right_wheel_ptr->_internal_pid_state) , right_wheel_ros_command.data, right_wheel_ptr->speed) ; 
  applyVoltageToWheel(left_wheel_ptr,left_wheel_ptr->_internal_pid_state.u_k) ; 
  applyVoltageToWheel(right_wheel_ptr,right_wheel_ptr->_internal_pid_state.u_k) ; 
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
  /**
   * Before updating the speed, let's compute the odometry first.
  */
  // odometry = computeOdometry(robot_pose,nh.now(),1/SPEED_UPDATING_FREQUENCY,left_wheel_ptr->speed,right_wheel_ptr->speed) ; 
  //Now we can update the speed 
  updateWheelSpeed(left_wheel_ptr);
  updateWheelSpeed(right_wheel_ptr);
  // now we update the data to be periodically sent to ros topics
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
  delay(5000) ; 
  Serial.begin(9600) ;
  Serial.print("Setting up wifi \n") ; 
  setupWiFi();
  nh.initNode() ;
  nh.getHardware()->setConnection(server, serverPort);
  nh.advertise(left_wheel_data_publisher);
  nh.advertise(right_wheel_data_publisher);
  nh.advertise(odometry_publisher) ; 
  nh.advertise(robot_pose_publisher) ; 
  nh.subscribe(pid_param_modification_sub) ; 
  nh.subscribe(left_wheel_command_sub);
  nh.subscribe(right_wheel_command_sub);
  left_wheel_data.header.frame_id = "left_wheel_frame"        ; right_wheel_data.header.frame_id = "right_wheel_frame"        ; 
  left_wheel_data.header.stamp = nh.now()                     ; right_wheel_data.header.stamp = nh.now()                      ; 
  left_wheel_data.vector.x = 0.0                              ; right_wheel_data.vector.x = 0.0                               ;
  left_wheel_data.vector.y = 0.0                              ; right_wheel_data.vector.y = 0.0                               ;
  left_wheel_data.vector.z = 0.0                              ; right_wheel_data.vector.z = 0.0                               ;
  left_wheel_ros_command.data = 0.0                           ; right_wheel_ros_command.data = 0.0                            ; 
  initWheel(left_wheel_ptr, 17,16,15,2,4,4,210) ;
  initWheel(right_wheel_ptr,12,13,27,26,14,14,210);  
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
                                    1000/(SPEED_UPDATING_FREQUENCY*portTICK_PERIOD_MS), //period of timer in ticks = 50HZ
                                    pdTRUE, // auto reload 
                                    (void*) 0, // timer index
                                    speedUpdatingTimerCallback // Callback function
                                    ) ; 
  bufferUpdatingTimer = xTimerCreate("bufferUpdatingTimer" , 
                                      1000/(BUFFER_UPDATING_FREQUENCY*portTICK_PERIOD_MS) , //period of timer in ticks = 100HZ
                                      pdTRUE , 
                                      (void*) 1,
                                      bufferUpdatingTimerCallback) ; 
  wheelCommandingTimer = xTimerCreate("wheelCommandingTimer" , 
                                      1000/(DEFAULT_WHEEL_COMMAND_FREQUENCY*portTICK_PERIOD_MS) , //period of timer in ticks = 100HZ
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
}

void loop() 
{
  left_wheel_data_publisher.publish(&left_wheel_data) ; 
  right_wheel_data_publisher.publish(&right_wheel_data) ;
  // odometry_publisher.publish(&odometry) ; 
  // robot_pose_publisher.publish(&robot_pose) ;
  nh.spinOnce();
  delay(uint32_t(1000/ROS_PUBLISHING_FREQUENCY)) ; 
}

