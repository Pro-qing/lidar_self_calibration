#ifndef LIDAR_SELF_CALIBRATION_HPP
#define LIDAR_SELF_CALIBRATION_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h> // 新增
#include <fstream>         // 新增
#include <vector>
#include <string>
#include <deque>

struct LaserLine {
    float avg_distance;
    float angle_center;
    int point_count;
    Eigen::Vector3f normal;
    std::string label;
    float confidence;
    std::vector<pcl::PointXYZ> raw_points;
    LaserLine() : avg_distance(0.0f), angle_center(0.0f), point_count(0), confidence(0.0f) {}
};

struct DetectedWall {
    std::string label;
    float avg_distance;
    Eigen::Vector3f pca_normal;
    int point_count;
    std::vector<pcl::PointXYZ> points;
};

struct CalibrationResult {
    Eigen::Matrix4f transform;
    float yaw;
    float right_error;
    float quality;      
};

class LidarCalibration {
public:
    LidarCalibration(ros::NodeHandle& nh);
    void run();
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_pub_, wall_points_pub_, calibrated_pub_, status_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string lidar_frame_, base_frame_, save_path_; // 新增 save_path_
    float actual_left_dist_, actual_right_dist_, actual_front_dist_, manual_lidar_height_;
    
    Eigen::Matrix4f transform_matrix_;
    bool calibration_done_;
    std::deque<CalibrationResult> calibration_queue_;
    int total_frames_;
    
    LaserLine left_line_, right_line_, front_line_;
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool detectThreeLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<DetectedWall> detectWallsWithRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    Eigen::Vector3f refineLineParametersPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices);
    void calculateCalibrationStep();
    void accumulateAndSmooth();
    void computeRobustAverage();
    void saveResultsToYaml(); // 新增保存函数
    
    void publishTF();
    void publishStatus();
    void publishWallPoints();
    void publishCalibratedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void printCalibrationResults();
    
    float normalizeAngleRad(float angle);
    float toDegrees(float radians);
    float toRadians(float degrees);
};

#endif