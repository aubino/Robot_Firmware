//This file contains the implementation of the PID i will be using for servoing
// We want to command the speed of our wheel so we will give the error as entry of the pid
// For example if our target is 3.14 rad/s and our current speed is 1.0 rad/s we will input 2.14 as entry of the function. 
// This will is simple proportional control and will be enough since our system does not have big dynamics. 
// The function will then output the voltage to apply.
//https://www.betzler.physik.uni-osnabrueck.de/Manuskripte/Elektronik-Praktikum/p3/doc2558.pdf gives a good overview of what a numerical pid is 
// 

#include <math.h>
#ifndef  SERVOING_H
#define  SERVOING_H
#define  PID_FREQUENCY 100
#define DEBUG_PID
#define MAX_I    1000.0
#define MIN_I   -1000.0


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
        double u_k , u_k_1  ; 
    #endif 
    PID_State(); 
} PID_State ; 

void updatePID(PID_State* pid_state_ptr,double command,double current_speed) ; 

///@brief implementation of the updating of the pid 
/// See https://engineering.stackexchange.com/questions/26537/what-is-a-definitive-discrete-pid-controller-equation for more details
void setPIDStateCoefficients(PID_State* pid_state_ptr, double kp , double ki, double kd,double ts) ; 

#endif