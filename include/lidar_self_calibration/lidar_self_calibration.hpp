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
#include <vector>
#include <string>
#include <deque>
#include <memory>
#include <algorithm>

// 激光线特征结构体
struct LaserLine {
    float avg_distance;     // 垂直距离
    float angle_center;     // 法向量角度（弧度）
    int point_count;        // 点数
    Eigen::Vector3f normal; // PCA计算的高精度法向量
    std::string label;      // front, left, right
    float confidence;       // 置信度
    std::vector<pcl::PointXYZ> raw_points; // 用于可视化的点
    
    LaserLine() : avg_distance(0.0f), angle_center(0.0f), point_count(0), confidence(0.0f) {}
};

// 墙面检测中间结果
struct DetectedWall {
    std::string label;
    float avg_distance;
    Eigen::Vector3f pca_normal;
    int point_count;
    std::vector<pcl::PointXYZ> points;
};

// 标定结果快照（用于队列平滑）
struct CalibrationResult {
    Eigen::Matrix4f transform;
    float yaw;          // 弧度
    float right_error;  // 左右墙一致性误差
    float quality;      
};

class LidarCalibration
{
public:
    LidarCalibration(ros::NodeHandle& nh);
    void run(); // 主循环
    
private:
    // --- ROS 通信 ---
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_pub_;      // 发布去地面后的点云
    ros::Publisher wall_points_pub_;   // 发布提取出的墙面点(彩色)
    ros::Publisher calibrated_pub_;    // 发布校准后的点云
    ros::Publisher status_pub_;        // 发布状态里程计
    
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // --- 参数变量 ---
    std::string lidar_frame_, base_frame_;
    
    // *** 关键：手动测量数据 ***
    float actual_left_dist_;   // Base_link 到左墙的距离
    float actual_right_dist_;  // Base_link 到右墙的距离
    float actual_front_dist_;  // Base_link 到前墙的距离
    float manual_lidar_height_;// Base_link 到雷达光心的垂直高度
    
    // --- 内部状态 ---
    Eigen::Matrix4f transform_matrix_;              // 当前计算出的变换矩阵
    bool calibration_done_;                         // 标志位
    std::deque<CalibrationResult> calibration_queue_; // 滑动窗口队列
    int total_frames_;
    
    // 存储当前帧检测到的线
    LaserLine left_line_, right_line_, front_line_;
    
    // --- 核心函数 ---
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    // 1. 墙面检测与 PCA 精修
    bool detectThreeLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<DetectedWall> detectWallsWithRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    Eigen::Vector3f refineLineParametersPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices);

    // 2. 标定计算
    void calculateCalibrationStep(); // 计算瞬时值
    
    // 3. 数据平滑
    void accumulateAndSmooth(); // 加入队列并计算鲁棒平均
    void computeRobustAverage();
    
    // 4. 工具与发布
    void publishTF();
    void publishStatus();
    void publishWallPoints();
    void publishCalibratedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void printCalibrationResults();
    
    // 辅助数学函数
    float normalizeAngleRad(float angle);
    float toDegrees(float radians);
    float toRadians(float degrees);
};

#endif