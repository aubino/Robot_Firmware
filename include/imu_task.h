#ifndef IMU_TASK_H
#define IMU_TAKS_H 
#include "freertos/task.h"
#include "freertos/timers.h"
#include "hal/i2c_hal.h"
#include "sensor_msgs/Imu.h"
#define IMU_TASK_FREQUENCY 30

sensor_msgs::Imu ros_imu_data ; 

/**
 * @brief Function to be called to initialize the imu in the setup part of the main file
 * @param imu_data_structure The data structure holding relevant infos like i2c port, used adresss , etc...
*/
void setupImu(void * imu_data_structure) ;  

/**
 * @brief Function to get imu data and transfert it to ros  sensor msg
 * @param imu_data_structure The data structure holding relevant infos like i2c port, used adresss , etc...
*/
sensor_msgs::Imu getImuData(void * imu_data_structure) ; 

/**
 * @brief The freertos routine task to execute. It should do in order 
 * - Get  raw data from imu_data_structure
 * - update ros_imu_data with that data
 * - delay for (1/IMU_TASK_FREQUENCY) seconds 
 * - repeat all over again.
*/
TaskFunction_t imuDataGetterTask(void * imu_data_structure) ;  

#endif
