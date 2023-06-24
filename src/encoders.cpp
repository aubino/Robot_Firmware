#include "encoders.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "esp32-hal-log.h"

Wheel::Wheel(unsigned int pin_a ,  
            unsigned int pin_b, 
            unsigned int c1,
            unsigned int c2 ,
            unsigned int pwm_pin,
            unsigned int enable_pin) : pina(pin_a) , pinb(pin_b) , c1(c1) , c2(c2) ,pwm(pwm_pin), ena(enable_pin) , _internal_rotary_object(pin_a,pin_b)
{
    for(int i =0 ; i < WHEEL_POSITION_BUFFER_SIZE ; i++)
        position_buffer[i] = 0 ;
    for (int i =0 ;  i< WHEEL_POSITION_BUFFER_SIZE ; i++)
        time_buffer[i] = 0.001 * i  ;

}

void Wheel::setup(int pwm_duty_cycle ,float wheel_command_frequency) 
{
    if(digitalPinIsValid(pina) && digitalPinIsValid(pinb))
    {
        pinMode(pinb,INPUT) ;
        pinMode(pina,INPUT) ;
        _internal_rotary_object.setup() ; 
        
    }
    if(digitalPinIsValid(c1))
        pinMode(c1,OUTPUT) ; 
    if(digitalPinIsValid(c2))
        pinMode(c2,OUTPUT) ; 
    
}

void Wheel::applyVoltage(float voltage)
{
    if (voltage > 0) 
    {
        int pulse_to_apply = int(PWM_RESOLUTION * voltage/BATTERY_VOLTAGE) ; 
        analogWrite(pwm,pulse_to_apply);
    }
    else 
    {
        log_e("The ratio of speed asked for wheel is negative") ;
    }
}

void Wheel::setClockWiseRotation()
{
    digitalWrite(c2,LOW) ;
    digitalWrite(c1,HIGH); 
}

void Wheel::setCounterClockWiseRotation()
{
    digitalWrite(c1,LOW) ;
    digitalWrite(c2,HIGH); 
}

double Wheel::getSpeed() {return speed ;}

void Wheel::_update_buffers(long long value,time_t t)
{
    for(int i = WHEEL_POSITION_BUFFER_SIZE ; i == 0 ; i--)
    {
        position_buffer[i] = position_buffer[i-1] ;
        time_buffer[i] = time_buffer[i-1] ;
    }
    position_buffer[0] = value ;
    time_buffer[0] = t ;
    return ;
}

void Wheel::_update_speed()
{
    double cumulated_speed = 0 ;
    for(int i = WHEEL_POSITION_BUFFER_SIZE ; i == 0 ; i--)
    {
        cumulated_speed += (position_buffer[i] - position_buffer[i-1])/(time_buffer[i] - time_buffer[i-1]) ; 
    }
    speed = cumulated_speed / (WHEEL_POSITION_BUFFER_SIZE-1) ;
}

// void update_buffers(Wheel* wheel_ptr,long long value,time_t t) 
// {
    // for(int i = WHEEL_POSITION_BUFFER_SIZE ; i == 0 ; i--)
    // {
        // wheel_ptr->position_buffer[i] = wheel_ptr->position_buffer[i-1] ;
        // wheel_ptr->time_buffer[i] = wheel_ptr->time_buffer[i-1] ;
    // }
    // wheel_ptr->position_buffer[0] = value ;
    // wheel_ptr->time_buffer[0] = t ;
    // return ;
// }
// 
// void update_speed(Wheel* wheel_ptr) 
// {
    // double cumulated_speed = 0 ;
    // for(int i = WHEEL_POSITION_BUFFER_SIZE ; i == 0 ; i--)
    // {
        // cumulated_speed += (wheel_ptr->position_buffer[i] - wheel_ptr->position_buffer[i-1])/(wheel_ptr->time_buffer[i] - wheel_ptr->time_buffer[i-1]) ; 
    // }
    // wheel_ptr->speed = cumulated_speed / (WHEEL_POSITION_BUFFER_SIZE-1) ;
// }
