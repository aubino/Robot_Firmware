#include "imu_task.h"

const int MPU_addr = 0x68 //I2C adress of the MPU-6050
int16_t AcX,AcY,AcZ,


sensor_msgs::Imu ros_imu_data;



void setupImu(void *imu_data_structure) {
    // Your implementation for IMU setup goes here
    // You can use the imu_data_structure to initialize the IMU
    // For example:
    imu_data_structure_t* imu_data = static_cast<imu_data_structure_t*>(imu_data_structure);
    imu_init(imu_data->i2c_port, imu_data->address);
}

sensor_msgs::Imu getImuData(void *imu_data_structure) {
    // Your implementation to get IMU data goes here
    // You can use the imu_data_structure to access the IMU and retrieve data
    // For example:
    imu_data_structure_t* imu_data = static_cast<imu_data_structure_t*>(imu_data_structure);
    imu_data_t imu_raw_data = imu_get_raw_data(imu_data->i2c_port);
    sensor_msgs::Imu imu_msg;
    imu_msg.linear_acceleration.x = imu_raw_data.acc_x;
    imu_msg.linear_acceleration.y = imu_raw_data.acc_y;
    imu_msg.linear_acceleration.z = imu_raw_data.acc_z;
    // ... (fill in other IMU data fields)
    return imu_msg;

    // For now, let's assume imu_msg is a pre-filled sensor_msgs::Imu with some dummy values:
    // return ros_imu_data;
}

void imuDataGetterTask(void *imu_data_structure) {
    while (1) {
        // Get IMU data
        ros_imu_data = getImuData(imu_data_structure);
        // Delay for (1/IMU_TASK_FREQUENCY) seconds
        vTaskDelay(pdMS_TO_TICKS(1000 / IMU_TASK_FREQUENCY));
    }
}

void imuSubCallback(const std_msgs::String::ConstrPtr& msg)
{
    ROS_INFO("Data Being Transmitted")
}

int main(int argc, char **argv[])
{
    nh.initNode(argc,argv,"Subber");
    ros::NodeHandle nh;
    
    
    int rate;
    string topic;

    ros::Subscriber sub_message = n.subscribe("imu_data_mpu_6050"),1000,imuSubCallback)
    ros::Rate r(rate);
    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep()
    }
    return 0
}
#endif
