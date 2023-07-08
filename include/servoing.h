//This file contains the implementation of the PID i will be using for servoing
// We want to command the speed of our wheel so we will give the error as entry of the pid
// For example if our target is 3.14 rad/s and our current speed is 1.0 rad/s we will input 2.14 as entry of the function. 
// This will is simple proportional control and will be enough since our system does not have big dynamics. 
// The function will then output the voltage to apply.
#include <math.h>
#define PID_P 100

float proportinnal_command(float error)
{
    return PID_P * error ; 
}