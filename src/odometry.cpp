#include "odometry.h"

void computeDirectKinematics(
    double theta ,
    double left_wheel_speed, 
    double right_wheel_speed,
    double& v_x,
    double& v_y,
    double& v_theta
)
{
    v_x     = 0.5 * WHEEL_RADIUS * (left_wheel_speed + right_wheel_speed) * cos(theta) ; 
    v_y     = 0.5 * WHEEL_RADIUS * (left_wheel_speed + right_wheel_speed) * sin(theta) ; 
    v_theta = (WHEEL_RADIUS/WHEEL_WIDTH) * (right_wheel_speed-left_wheel_speed)        ;
    return ; 
}

void computeInvertKinematics(
    double theta ,
    double v_x ,
    double v_y , 
    double v_theta, 
    double& left_wheel_speed , 
    double& right_wheel_speed
) 
{
    if(abs(v_theta)>1.0e-4)
    {
        double v = sqrt(v_x*v_x + v_y*v_y) ; 
        double R_t = v/v_theta ;
        right_wheel_speed = (1+(ROBOT_WIDTH/(2*R_t))) * (v /WHEEL_RADIUS) ;  
        left_wheel_speed = (1-(ROBOT_WIDTH/(2*R_t))) * (v /WHEEL_RADIUS) ;
        return ; 
    }
    else
    {
        // if v_theta is less than 0.0001 we consider it being null
        double v = sqrt(v_x*v_x + v_y*v_y)  ; 
        left_wheel_speed = v / WHEEL_RADIUS ; 
        right_wheel_speed = v /WHEEL_RADIUS ; 
        return ; 
    }
}

nav_msgs::Odometry computeOdometry(
    geometry_msgs::Pose2D& p,
    ros::Time t ,  
    double dt ,
    double left_wheel_speed ,
    double right_wheel_speed
) 
{
    double vx(0.0), vy(0.0),vth(0.0) ; 
    computeDirectKinematics(p.theta ,left_wheel_speed,right_wheel_speed,vx,vy,vth) ; 
    double delta_x = (vx * cos(p.theta) - vy * sin(p.theta)) * dt;
    double delta_y = (vx * sin(p.theta) + vy * cos(p.theta)) * dt;
    double delta_th = vth * dt;
    nav_msgs::Odometry result_odom  ; 
    result_odom.header.frame_id = "odom" ; 
    result_odom.header.stamp = t ;
    result_odom.child_frame_id = "base_link" ; 
    result_odom.pose.pose.position.x = p.x + delta_x ; 
    result_odom.pose.pose.position.y = p.y + delta_y ; 
    result_odom.pose.pose.position.z = 0.0 ; 
    result_odom.twist.twist.linear.x = vx;
    result_odom.twist.twist.linear.y = vy;
    result_odom.twist.twist.angular.z = vth;
    // Now we update the new Pose
    p.x += delta_x ; 
    p.y += delta_y ; 
    p.theta += delta_th ; 
    return result_odom ; 
}