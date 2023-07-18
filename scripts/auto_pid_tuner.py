import rospy 
import csv 
import matplotlib
from geometry_msgs.msg import Vector3, Vector3Stamped
from std_msgs.msg import Float64
import numpy as np 
import rosbag
import pandas as pd
import yaml
import os 
saving_dir = "/home/aubin/pid_report"
Kp_min = -6.0
Kp_max = -0.2
Kp_step = 0.1
Ki_min = 0
Ki_max = 5 
Ki_step = 0.1
Kd_min = -5.0
Kd_max = 10 
Kd_step = 0.01

# Kp_range = list(np.arange(Kp_min,Kp_max,Kp_step))
# Ki_range = list(np.arange(Ki_min,Ki_max,Ki_step))
kd = 0.0
# kd_range = list(range(-5.0,10,0.01))

speeds  = []
times   = []

def register_speed(data : Vector3Stamped) : 
    speeds.append(data.vector.x)
    times.append(data.header.stamp.secs * 1000000 + data.header.stamp.nsecs)
    

def mainloop() : 
    sub = rospy.Subscriber("/left_wheel/command/get", Vector3Stamped, register_speed,queue_size=2)
    pid_pub  = rospy.Publisher("/PID/set",Vector3,queue_size=2)
    command_pub  = rospy.Publisher("/left_wheel/command/set",Float64,queue_size=2)
    rospy.init_node('auto_pid_tuner', anonymous=False)
    rate = rospy.Rate(0.04) # every 40 sec
    for kp in np.arange(Kp_min,Kp_max,Kp_step) : 
        for ki in np.arange(Ki_min,Ki_max,Ki_step) : 
            pid_pub.publish(Vector3(x=kp,y=ki,z=kd))
            command_pub.publish(Float64(4*3.1415))
            rate.sleep()
            np_speeds = np.array(speeds)
            np_times = np.array(times)
            command_pub.publish(Float64(0))
            speeds.clear()
            times.clear()
            rate.sleep()
            np_speeds_file_name = f"Kp_{kp}_Ki{ki}"
            np_times_file_name = f"Kp_{kp}_Ki{ki}"
            np.save(os.path.join(saving_dir,np_speeds_file_name,"speeds"),np_speeds)
            np.save(os.path.join(saving_dir,np_times_file_name,"times"),np_times)
    command_pub.publish(Float64(0))
    
if __name__ == '__main__':
     try:
         mainloop()
     except rospy.ROSInterruptException as e:
         print(e)
         pass