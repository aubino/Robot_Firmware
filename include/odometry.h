
#include "geometry.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include "ros/time.h"
#include "std_msgs/Time.h"
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


/**
 * @brief Compute the odometry of the robot using the example at http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
 * @param[in,out] p The current position  of the robot on  X and Y axis  (m)
 * @param[in] dt Elapsed time since last odometry computation (s)
 * @param[in] left_wheel_speed Current speed of the left wheel (rad/s)
 * @param[in] right_wheel_speed Current speed of the right wheel (rad/s)
 * @returns The computed speed in the form of nav_msgs::Odometry
*/
nav_msgs::Odometry computeOdometry(
    geometry_msgs::Pose2D& p ,
    ros::Time t ,
    double dt ,
    double left_wheel_speed ,
    double right_wheel_speed
) ; 

/**
    @brief Used to compute the inverse kinematics of the robot ie given the linear speed, 
        what wheel speed should be given to each wheel  
    @param[in] theta Current orientation of the robot
    @param[in] v_x Desired Speed along X axis
    @param[in] v_y Desired speed along Y axis
    @param[in] v_theta Desired rotation speed of the robot
    @param[out] left_wheel_speed Speed to apply to left wheel 
    @param[out] right_wheel_speed Speed to apply to the right wheel
*/
void computeInvertKinematics(
    double theta ,
    double v_x ,
    double v_y , 
    double v_theta, 
    double& left_wheel_speed , 
    double& right_wheel_speed
) ; 

#endif