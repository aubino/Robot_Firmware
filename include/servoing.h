//This file contains the implementation of the PID i will be using for servoing
// We want to command the speed of our wheel so we will give the error as entry of the pid
// For example if our target is 3.14 rad/s and our current speed is 1.0 rad/s we will input 2.14 as entry of the function. 
// This will is simple proportional control and will be enough since our system does not have big dynamics. 
// The function will then output the voltage to apply.
//https://www.betzler.physik.uni-osnabrueck.de/Manuskripte/Elektronik-Praktikum/p3/doc2558.pdf gives a good overview of what a numerical pid is 
// 
#include <math.h>
#include "encoders.h"
#ifndef servoing_h
#define servoing_h
#define PID_FREQUENCY 100
// #define PID_P -0.90
// #define PID_I  0.60
// #define PID_D  0
#define MAX_I    1000.0
#define MIN_I   -1000.0
double PID_P(0) ,PID_I(0) ,  PID_D(0) ; 

double pid_command(double order, double last_speed, double current_speed,double * error_sum)
{
    double Kp = PID_P ; 
    double Ki = (Kp * PID_I)/(PID_FREQUENCY) ; 
    double Kd = (Kp * PID_D)/(PID_FREQUENCY) ; 
    double error = current_speed - order ; 
    double derivate = current_speed -  last_speed ; 
    *error_sum += error ; 
    //return Kp * error + Ki * max(min(*error_sum,MAX_I),MIN_I) + Kd * derivate; 
    return Kp * error + Ki * (*error_sum) + Kd * derivate;
}

/// @brief This is a simple structure to keep track of the pid  command structure.
/// It will directly be integrated into the Wheel data structure and serve as a "private member" 
/// only acessed when updating the commands to be sent to the wheels and at the initialization. 
typedef struct PID_State
{
    
    double proportional_coefficient ; 
    double integral_coefficient     ; 
    double derivate_coefficient     ; 
    double sampling_interval        ; // in seconds 
    #ifdef DEBUG_PID
        double i_k                      ; 
        double d_k                      ; 
        double u_k_1 , u_k              ; 
        double e_k_1 , e_k              ; 
    #else
        double e_k , e_k_1 , e_k_2 ;  
        double u_k_1 , u_k ; 
    #endif 
    PID_State() {} ; 
} PID_State ; 

void updatePID(PID_State* pid_state_ptr,double command,double current_speed)
{
    ///@brief implementation of the updating of the pid 
    /// See https://engineering.stackexchange.com/questions/26537/what-is-a-definitive-discrete-pid-controller-equation for more details
    
    #ifdef DEBUG_PID
        pid_state_ptr->e_k_1 = pid_state_ptr->e_k      ; 
        pid_state_ptr->e_k   = command - current_speed ;
        pid_state_ptr->i_k +=  pid_state_ptr->sampling_interval * (pid_state_ptr->e_k + pid_state_ptr->e_k_1) / 2 ;
        pid_state_ptr->d_k = (pid_state_ptr->e_k - pid_state_ptr->e_k_1) / pid_state_ptr->sampling_interval ; 
        pid_state_ptr->u_k_1 = pid_state_ptr->u_k ; 
        pid_state_ptr->u_k = pid_state_ptr->proportional_coefficient * pid_state_ptr->e_k + pid_state_ptr->integral_coefficient * pid_state_ptr->i_k + pid_state_ptr->derivate_coefficient * pid_state_ptr->d_k ; 
    
    #else
        pid_state_ptr->u_k_1 = pid_state_ptr->u_k ; 
        
        pid_state_ptr->u_k  += (pid_state_ptr->proportional_coefficient + 
                                (pid_state_ptr->integral_coefficient * pid_state_ptr->sampling_interval)/2 + 
                                pid_state_ptr->derivate_coefficient/pid_state_ptr->sampling_interval) * pid_state_ptr->e_k      ;
        
        pid_state_ptr->u_k  += (-pid_state_ptr->proportional_coefficient + 
                                (pid_state_ptr->integral_coefficient * pid_state_ptr->sampling_interval)/2 - 
                                (2*pid_state_ptr->derivate_coefficient/pid_state_ptr->sampling_interval)) * pid_state_ptr->e_k_1 ; 
        
        pid_state_ptr->u_k  += (pid_state_ptr->derivate_coefficient/pid_state_ptr->sampling_interval) * pid_state_ptr->e_k_2    ; 

    #endif


}

void setPIDStateCoefficients(PID_State* pid_state_ptr, double kp , double ki, double kd,double ts)
{
    pid_state_ptr->proportional_coefficient = kp ; 
    pid_state_ptr->integral_coefficient = ki ; 
    pid_state_ptr->derivate_coefficient = kd ; 
    pid_state_ptr->sampling_interval = ts ; 
}

#endif