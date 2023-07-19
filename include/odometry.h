
#include "geometry.h"
#include <math.h>
#ifndef ODOMETRY_H
#define ODOMETRY_H
/// @brief Used to compute the direct kinematics. 
///   Go to https://www.roboticsbook.org/S52_diffdrive_actions.html for more informations about the direct kinematics model. 
/// @param theta 
/// @param left_wheel_speed 
/// @param right_wheel_speed 
/// @param v_x 
/// @param v_y 
/// @param v_theta 
void computeDirectKinematics(
    double theta ,
    double left_wheel_speed, 
    double right_wheel_speed,
    double& v_x,
    double& v_y,
    double& v_theta
) ;

void computeOdometry(
    double x , 
    double y ,
    double theta , 
    double dt ,
    double left_wheel_speed ,
    double right_wheel_speed
) ; 

void computeInvertKinematics(
    double theta ,
    double v_x ,
    double v_y , 
    double v_theta, 
    double& left_wheel_speed , 
    double& right_wheel_speed
) ; 

#endif