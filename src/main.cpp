#include "lidar_self_calibration/lidar_self_calibration.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_self_calibration_node");
    ros::NodeHandle nh("~");
    
    LidarCalibration calib(nh);
    std::cout << "PCL版本: " << PCL_VERSION << std::endl;
    ROS_INFO("Starting lidar calibration node...");
    calib.run();
    
    return 0;
}