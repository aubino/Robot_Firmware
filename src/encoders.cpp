#include "encoders.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "esp32-hal-log.h"


void initWheel(
    Wheel * wheel_ptr , 
    unsigned int pin_a ,  
    unsigned int pin_b, 
    unsigned int c1,
    unsigned int c2 ,
    unsigned int pwm_pin,
    unsigned int enable_pin,
    double reduction)
{
    wheel_ptr->pina = pin_a ; 
    wheel_ptr->pinb = pin_b ; 
    wheel_ptr->c1 = c1 ; 
    wheel_ptr->c2 = c2 ;
    wheel_ptr->pwm = pwm_pin ;
    wheel_ptr->ena = enable_pin ; 
    wheel_ptr->_internal_rotary_object = Rotary(pin_a,pin_b) ; 
    initCircularBuffer(&wheel_ptr->position_buffer,WHEEL_POSITION_BUFFER_SIZE) ;
    initTimeCircularBuffer(&wheel_ptr->timer_buffer,WHEEL_POSITION_BUFFER_SIZE) ; 
    wheel_ptr->encoder_position = 0 ;
    wheel_ptr->reduction_factor = reduction ;  
    wheel_ptr->speed = 0 ; 
    wheel_ptr->minimum_coder_tick_to_compute_speed = MINIMUM_SPEED_TICK_TO_COMPUTE_SPEED ; 
    return ; 
}

double getRadiantPosition(Wheel * wheel_ptr) 
{
    return (wheel_ptr->encoder_position/wheel_ptr->reduction_factor) * 2 * MATH_PI ;
}

void initCircularBuffer(CircularBuffer* circular_buffer_ptr,size_t size)
{
    circular_buffer_ptr->current_index = 0 ; 
    circular_buffer_ptr->buffer_size = size ; 
    for(int i =0 ; i<circular_buffer_ptr->buffer_size ; i++)
        circular_buffer_ptr->internal_buffer[i] = 0 ;
}

void initTimeCircularBuffer(TimeCircularBuffer* time_circular_buffer_ptr,size_t size) 
{
    time_circular_buffer_ptr->current_index = 0 ; 
    time_circular_buffer_ptr->buffer_size = size ; 
    for(int i =0 ; i<time_circular_buffer_ptr->buffer_size ; i++)
        time_circular_buffer_ptr->internal_buffer[i] = 0 ;
}

void setup_wheel(Wheel * wheel_ptr,int pwm_duty_cycle,float wheel_command_frequency)
{
    if(digitalPinIsValid(wheel_ptr->pina) && digitalPinIsValid(wheel_ptr->pinb))
    {
        pinMode(wheel_ptr->pinb,INPUT) ;
        pinMode(wheel_ptr->pina,INPUT) ;
        wheel_ptr->_internal_rotary_object.setup() ; 
        
    }
    if(digitalPinIsValid(wheel_ptr->c1))
        pinMode(wheel_ptr->c1,OUTPUT) ; 
    if(digitalPinIsValid(wheel_ptr->c2))
        pinMode(wheel_ptr->c2,OUTPUT) ; 
}

void setWheelClockWiseRotation(Wheel * wheel_ptr) 
{
    digitalWrite(wheel_ptr->c1,LOW) ;
    digitalWrite(wheel_ptr->c2,HIGH);
}

void setWheelCounterClockWiseRotation(Wheel * wheel_ptr) 
{
    digitalWrite(wheel_ptr->c1,LOW) ;
    digitalWrite(wheel_ptr->c2,HIGH); 
}

void applyVoltageToWheel(Wheel * wheel_ptr,float voltage)
{
    int pulse_to_apply = int(abs(PWM_RESOLUTION * voltage/BATTERY_VOLTAGE)) ; 
    if (voltage > 0) 
    {
        setWheelClockWiseRotation(wheel_ptr) ; 
        analogWrite(wheel_ptr->pwm,pulse_to_apply);
    }
    else 
    {
        setWheelCounterClockWiseRotation(wheel_ptr) ;
        analogWrite(wheel_ptr->pwm,pulse_to_apply) ; 
    }
}

void  IRAM_ATTR updateWheelBuffers(Wheel * wheel_ptr)
{
    wheel_ptr->position_buffer.push(wheel_ptr->encoder_position) ;
    wheel_ptr->timer_buffer.push(millis()) ; 
}

void stopWheel(Wheel * wheel_ptr) 
{
    digitalWrite(wheel_ptr->ena,LOW);
}

void IRAM_ATTR onWheelInterrupt(Wheel* wheel_ptr)
{
    unsigned char result = wheel_ptr->_internal_rotary_object.process()  ;
    if (result == DIR_CW)
    {
        portENTER_CRITICAL_ISR(&spinlock) ; 
        wheel_ptr->encoder_position ++ ;
        portEXIT_CRITICAL_ISR(&spinlock) ; 
    }
    else if (result == DIR_CCW)
    {
        portENTER_CRITICAL_ISR(&spinlock) ; 
        wheel_ptr->encoder_position -- ;
        portEXIT_CRITICAL_ISR(&spinlock) ; 
    } 
}

void  IRAM_ATTR updateWheelSpeed(Wheel * wheel_ptr)
{
    int min_diff_index = wheel_ptr->position_buffer.current_index == wheel_ptr->position_buffer.buffer_size-1 ? 0 : wheel_ptr->position_buffer.current_index +1 ; 
    for(int i = 0  ;  i<wheel_ptr->position_buffer.buffer_size ; i++)
    {
        if((wheel_ptr->position_buffer.internal_buffer[i] - wheel_ptr->position_buffer.internal_buffer[wheel_ptr->position_buffer.current_index])>=wheel_ptr->minimum_coder_tick_to_compute_speed)
        {
            min_diff_index = i ;
            double tick_speed = (wheel_ptr->position_buffer.internal_buffer[wheel_ptr->position_buffer.current_index] - wheel_ptr->position_buffer.internal_buffer[min_diff_index]) / (wheel_ptr->timer_buffer.internal_buffer[wheel_ptr->position_buffer.current_index] - wheel_ptr->timer_buffer.internal_buffer[min_diff_index]) ; 
            double speed = 2 * MATH_PI * tick_speed / wheel_ptr->reduction_factor ;
            portENTER_CRITICAL(&spinlock) ; 
            wheel_ptr->speed = speed ; 
            portEXIT_CRITICAL(&spinlock) ; 
            return ; 
        }
    }
}

Wheel::Wheel(){}

CircularBuffer::CircularBuffer(){} 

TimeCircularBuffer::TimeCircularBuffer(){}





