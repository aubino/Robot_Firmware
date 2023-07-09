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
#define PID_FREQUENCY 50
#define PID_P 100
#define PID_I 20
#define PID_D 10
#define MAX_I 200.0
#define MIN_I 200.0

double proportinnal_command(double order, double last_speed, double current_speed,double * error_sum)
{
    constexpr double Kp = PID_P ; 
    constexpr double Ki = (Kp * PID_I)/(PID_FREQUENCY) ; 
    constexpr double Kd = (Kp * PID_D)/(PID_FREQUENCY) ; 
    double error = current_speed - order ; 
    double derivate = current_speed -  last_speed ; 
    *error_sum += error ; 
    return Kp * error + Ki * max(min(*error_sum,MAX_I),MIN_I) + Kd * derivate; 
}


#endif