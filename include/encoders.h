#define WHEEL_POSITION_BUFFER_SIZE 20
#define PWM_RESOLUTION 255
#define BATTERY_VOLTAGE 12
#define DEFAULT_BAUD_RATE 9600 
#define DEFAULT_WHEEL_COMMAND_FREQUENCY 50
#define DEFAULT_PWM_DUTY_CYCLE 2000
#include <time.h>
#include "Rotary.h"

#ifndef encoders_h
#define encoders_h


class Wheel 
{
    public  :
        Wheel(unsigned int pin_a ,  
            unsigned int pin_b, 
            unsigned int c1,
            unsigned int c2 ,
            unsigned int pwm_pin,
            unsigned int enable_pin);
        void setup(int pwm_duty_cycle = DEFAULT_PWM_DUTY_CYCLE ,float wheel_command_frequency = DEFAULT_WHEEL_COMMAND_FREQUENCY) ; 
        double getSpeed(); 
        void setClockWiseRotation(); 
        void setCounterClockWiseRotation() ;
        void applyVoltage(float ); 
        void _update_buffers(long long value,time_t t);
        void _update_speed(); 
        void start();
        void stop() ;
        void _on_change() ; 
        uint8_t getPinA() ;
        uint8_t getPinB() ;

    private : 
        volatile long long encoder_position;
        double speed ; 
        unsigned int pina ; // encoder a channel
        unsigned int pinb ; // encoder b channel
        unsigned int c1 ; // pin to enable rotation clockwise
        unsigned int c2 ; // pin to enable rotation counter clockwise
        unsigned int pwm ; // pin to apply pwm to the wheel 
        unsigned int ena ; 
        Rotary _internal_rotary_object ; 
        long long position_buffer[WHEEL_POSITION_BUFFER_SIZE] ; 
        time_t time_buffer[WHEEL_POSITION_BUFFER_SIZE] ;
} ;

// typedef struct Wheel
// {
    // volatile long long encoder_position; 
    // 
    // double speed ;
    // unsigned int _pina ; // encoder a channel
    // unsigned int _pinb ; // encoder b channel
    // unsigned int _c1 ; // pin to enable rotation clockwise
    // unsigned int _c2 ; // pin to enable rotation counter clockwise
    // unsigned int _pwm ; // pin to apply pwm to the wheel 
    // long long position_buffer[WHEEL_POSITION_BUFFER_SIZE] ; 
    // time_t time_buffer[WHEEL_POSITION_BUFFER_SIZE] ;
// } Wheel ;
// 
// void update_buffers(Wheel* wheel_ptr,long long value,time_t t);
// 
// void update_speed(Wheel* wheel_ptr); 
// 
#endif
