#include "encoders.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "esp32-hal-log.h"

Wheel::Wheel(unsigned int pin_a ,  
            unsigned int pin_b, 
            unsigned int c1,
            unsigned int c2 ,
            unsigned int pwm_pin,
            unsigned int enable_pin,
            double reduction) : 
                pina(pin_a) , 
                pinb(pin_b) , 
                c1(c1) , 
                c2(c2) ,
                pwm(pwm_pin), 
                ena(enable_pin) , 
                _internal_rotary_object(pin_a,pin_b), 
                position_buffer(WHEEL_POSITION_BUFFER_SIZE),
                timer_buffer(WHEEL_POSITION_BUFFER_SIZE),
                encoder_position{},
                reduction_factor(reduction)
{
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

void IRAM_ATTR Wheel::_update_buffers()
{
    position_buffer.push(encoder_position) ;
    timer_buffer.push(millis()) ; 
    return ;
}

void Wheel::_update_speed()
{
    // speed is in radiant per second
    // we first compute the time difference for each buffer position of the buffer (in second)
    double  inv_dt = DEFAULT_WHEEL_COMMAND_FREQUENCY ; // (speed = (position1-position0)/ dt) 
    // the buffer has two potential parts
    // from 0 to current_index
    // And from current_index to size
    // so we walk through it in two steps 
    // first from current_index to 0 
    // and then from size-1 to current_index +1
    double cumulated_speed = 0 ;
    for(int i =position_buffer.current_index ; i>=1   ; i--)
        cumulated_speed += (position_buffer.internal_buffer[i]-position_buffer.internal_buffer[i-1]) * inv_dt ; 
    for(int i = position_buffer.buffer_size-1 ; i > position_buffer.current_index ; i--)
        cumulated_speed += (position_buffer.internal_buffer[i] - position_buffer.internal_buffer[i-1]) * inv_dt ;
    double speed_in_tick_per_second = cumulated_speed /position_buffer.buffer_size ; // here is the pure speed in ticks per second. since speed_in_tick_per_second/speed = reduction 
    double speed_in_turn_per_second = speed_in_tick_per_second / reduction_factor ; 
    speed = speed_in_turn_per_second * 2 * MATH_PI ;  // and there is speed in rad/s
}

void Wheel::start() { digitalWrite(ena,HIGH);}

void Wheel::stop() { digitalWrite(ena,LOW);}

uint8_t Wheel::getPinA() {return pina ; }

uint8_t Wheel::getPinB() {return pinb ; }

long long int  Wheel::getPosition(){return encoder_position ; }

void IRAM_ATTR Wheel::_on_change()
{
    unsigned char result = _internal_rotary_object.process()  ;
    if (result == DIR_CW)
        encoder_position ++ ;
    else if (result == DIR_CCW) 
        encoder_position -- ;
}

