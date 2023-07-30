#include "servoing.h"

PID_State::PID_State(){} ; 

void updatePID(PID_State* pid_state_ptr,double command,double current_speed)
{
    
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
