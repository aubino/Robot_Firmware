#define WHEEL_POSITION_BUFFER_SIZE 16
#define MAX_BUFFER_SIZE 20
#define PWM_RESOLUTION 255 //bits
#define BATTERY_VOLTAGE 11.1
#define DEFAULT_BAUD_RATE 9600 
#define DEFAULT_WHEEL_COMMAND_FREQUENCY 100
#define MATH_PI  3.14159265359
#define DEFAULT_PWM_DUTY_CYCLE 2000 //ms
#define REPORT_TIME 1 
#define MINIMUM_SPEED_TICK_TO_COMPUTE_SPEED 5
#include <time.h>
#include "Rotary.h"
#include "cmath"
#include "freertos/semphr.h"
//constexpr int  PWM_MAX_VALUE = std::pow(2,PWM_RESOLUTION) - 1 ; 


#ifndef encoders_h
#define encoders_h
//#define DEBUG_MODE 
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED ; 

typedef struct CircularBuffer
{
    volatile long long int   internal_buffer[MAX_BUFFER_SIZE];
    int current_index ;
    size_t buffer_size ;
    CircularBuffer() ; 
    CircularBuffer(size_t size) : current_index(0),buffer_size(size) , internal_buffer{}
    {

    }
    void push(long long int value)
    {
        if(current_index == buffer_size)
            current_index = 0 ;
        else
            current_index ++ ;
        internal_buffer[current_index] = value ; 
    }

} CircularBuffer ; 

typedef struct TimeCircularBuffer
{   
    //TickType_t  internal_buffer[MAX_BUFFER_SIZE];
    unsigned long  internal_buffer[MAX_BUFFER_SIZE];
    int current_index ;
    size_t buffer_size ;
    TimeCircularBuffer() ; 
    TimeCircularBuffer(size_t size) : current_index(0),buffer_size(size) , internal_buffer{}
    {}
    // void push(TickType_t value)
    void push(unsigned long value)
    {
        if(current_index == buffer_size)
            current_index = 0 ;
        else
            current_index ++ ;
        internal_buffer[current_index] = value ; 
    }
} TimeCircularBuffer ; 

typedef struct Wheel
{
    volatile long long int encoder_position;
    double speed ; 
    double reduction_factor ; 
    unsigned int pina ; // encoder a channel
    unsigned int pinb ; // encoder b channel
    unsigned int c1 ; // pin to enable rotation clockwise
    unsigned int c2 ; // pin to enable rotation counter clockwise
    unsigned int pwm ; // pin to apply pwm to the wheel 
    unsigned int ena ; 
    Rotary _internal_rotary_object ;
    unsigned int minimum_coder_tick_to_compute_speed ;
    CircularBuffer position_buffer ;
    TimeCircularBuffer timer_buffer ;
    Wheel() ; 
} Wheel ; 

void setup_wheel(Wheel * wheel_ptr,int pwm_duty_cycle = DEFAULT_PWM_DUTY_CYCLE ,float wheel_command_frequency = DEFAULT_WHEEL_COMMAND_FREQUENCY) ; 
void setWheelClockWiseRotation(Wheel * wheel_ptr); 
void setWheelCounterClockWiseRotation(Wheel * wheel_ptr) ;
void applyVoltageToWheel(Wheel * wheel_ptr,float );
void stopWheel(Wheel * wheel_ptr)  ; 
void  updateWheelBuffers(Wheel * wheel_ptr); 
void  IRAM_ATTR updateWheelSpeed(Wheel * wheel_ptr);
void IRAM_ATTR onWheelInterrupt(Wheel * wheel_ptr) ; 
void initWheel(
    Wheel * wheel_ptr , 
    unsigned int pin_a ,  
    unsigned int pin_b, 
    unsigned int c1,
    unsigned int c2 ,
    unsigned int pwm_pin,
    unsigned int enable_pin,
    double reduction) ; 
double getRadiantPosition(Wheel * wheel_ptr) ; 
void initCircularBuffer(CircularBuffer* circular_buffer_ptr,size_t size) ;
void initTimeCircularBuffer(TimeCircularBuffer* time_circular_buffer_ptr,size_t size) ; 
void printWheelState(Wheel* wheel_ptr);

#endif
