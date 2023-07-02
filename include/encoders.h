#define WHEEL_POSITION_BUFFER_SIZE 16
#define MAX_BUFFER_SIZE 20
#define PWM_RESOLUTION 255
#define BATTERY_VOLTAGE 11.1
#define DEFAULT_BAUD_RATE 9600 
#define DEFAULT_WHEEL_COMMAND_FREQUENCY 100
#define MATH_PI  3.14159265359
#define DEFAULT_PWM_DUTY_CYCLE 2000 //ms
#include <time.h>
#include "Rotary.h"

#ifndef encoders_h
#define encoders_h
//#define DEBUG_MODE 


typedef struct CircularBuffer
{
    volatile long long int   internal_buffer[MAX_BUFFER_SIZE];
    int current_index ;
    size_t buffer_size ;
    CircularBuffer(size_t size) : current_index(0),buffer_size(size) , internal_buffer({0})
    {

    }
    void push(long long int value)
    {
        if(current_index == WHEEL_POSITION_BUFFER_SIZE)
            current_index = 0 ;
        else
            current_index ++ ;
        internal_buffer[current_index] = value ; 
    }

} CircularBuffer ; 


class Wheel 
{
    public  :
        Wheel(unsigned int pin_a ,  
            unsigned int pin_b, 
            unsigned int c1,
            unsigned int c2 ,
            unsigned int pwm_pin,
            unsigned int enable_pin,
            double reduction = 1 );
        void setup(int pwm_duty_cycle = DEFAULT_PWM_DUTY_CYCLE ,float wheel_command_frequency = DEFAULT_WHEEL_COMMAND_FREQUENCY) ; 
        double getSpeed(); 
        void setClockWiseRotation(); 
        void setCounterClockWiseRotation() ;
        void applyVoltage(float ); 
        void  IRAM_ATTR _update_buffers();
        void _update_speed(); 
        void start();
        void stop() ;
        void IRAM_ATTR _on_change() ; 
        uint8_t getPinA() ;
        uint8_t getPinB() ;
        long long int  getPosition() ;
        CircularBuffer position_buffer ;

    private : 
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
} ;


#endif
