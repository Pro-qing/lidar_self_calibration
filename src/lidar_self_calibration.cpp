#include "lidar_self_calibration/lidar_self_calibration.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <clocale>
#include <cmath>
#include <iomanip>

// 构造函数
LidarCalibration::LidarCalibration(ros::NodeHandle& nh) : nh_(nh)
{
    std::setlocale(LC_ALL, ""); // 支持中文输出
    
    // 读取坐标系名称
    nh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    
    // =================================================================================
    // [用户配置区域] 请确保这里的值（或Launch文件中的值）与实际手动测量一致
    // 单位：米
    // =================================================================================
    nh_.param<float>("actual_left_distance", actual_left_dist_, 0.0f);   // 车辆中心到左墙
    nh_.param<float>("actual_right_distance", actual_right_dist_, 0.0f);// 车辆中心到右墙
    nh_.param<float>("actual_front_distance", actual_front_dist_, 0.0f); // 车辆中心到前墙
    nh_.param<float>("manual_lidar_height", manual_lidar_height_, 1.0f); // 雷达安装高度
    // =================================================================================

    // 订阅与发布
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_16", 1, &LidarCalibration::pointCloudCallback, this);
    
    filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
    calibrated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("calibrated_points", 1);
    wall_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_points", 1);
    status_pub_ = nh_.advertise<nav_msgs::Odometry>("calibration_status", 1);
    
    // 初始化
    calibration_done_ = false;
    transform_matrix_.setIdentity();
    total_frames_ = 0;
    
    ROS_INFO("================================================");
    ROS_INFO("      自动标定节点已启动 (Release Version)");
    ROS_INFO("      用户参数配置:");
    ROS_INFO("      Front Dist: %.3f m", actual_front_dist_);
    ROS_INFO("      Left  Dist: %.3f m", actual_left_dist_);
    ROS_INFO("      Right Dist: %.3f m", actual_right_dist_);
    ROS_INFO("================================================");
}

// 主回调函数
void LidarCalibration::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    total_frames_++;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty()) return;

    // --- 步骤 1: Z轴直通滤波 (关键) ---
    // 作用：剔除地面和天花板，防止干扰RANSAC直线拟合
    // 范围：-1.5m 到 +1.5m (根据雷达安装高度可微调)
    pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.5, 1.5);
    pass.filter(*z_filtered_cloud);

    // 发布滤波后的点云供调试
    sensor_msgs::PointCloud2 debug_msg;
    pcl::toROSMsg(*z_filtered_cloud, debug_msg);
    debug_msg.header = msg->header;
    filtered_pub_.publish(debug_msg);
    
    // --- 步骤 2: 墙面检测 ---
    bool detection_success = detectThreeLines(z_filtered_cloud);
    
    if (detection_success) {
        // 可视化提取出的墙面点
        publishWallPoints();
        
        // --- 步骤 3: 标定计算 ---
        calculateCalibrationStep(); // 计算瞬时值
        accumulateAndSmooth();      // 滑动窗口平滑
        
        // 发布状态
        publishStatus();
        
        // 如果计算完成，发布TF和最终点云
        if (calibration_done_) {
            publishTF();
            publishCalibratedCloud(z_filtered_cloud); // 使用Z过滤后的点云验证对齐效果
            
            // 降低打印频率，避免刷屏
            if (total_frames_ % 50 == 0) {
                printCalibrationResults();
            }
        }
    } 
}

// PCA 精修算法：计算点云簇的最优拟合平面(直线)法向量
Eigen::Vector3f LidarCalibration::refineLineParametersPCA(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    const std::vector<int>& indices) 
{
    if (indices.size() < 5) return Eigen::Vector3f::Zero();

    // 1. 计算质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, indices, centroid);

    // 2. 计算协方差矩阵
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrix(*cloud, indices, centroid, covariance_matrix);

    // 3. 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
    
    // 最小特征值对应的特征向量即为法向量
    Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0);
    
    // 强制法向量在水平面 (假设雷达大致水平安装)
    normal.z() = 0.0f;
    normal.normalize();

    // 统一法向量方向：指向原点
    Eigen::Vector3f centroid_vec = centroid.head<3>();
    if (normal.dot(centroid_vec) > 0) {
        normal = -normal;
    }

    // 计算原点到直线的垂直距离
    float distance = std::abs(normal.dot(centroid_vec));
    return Eigen::Vector3f(normal.x(), normal.y(), distance);
}

// 检测三面墙
bool LidarCalibration::detectThreeLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud->empty()) return false;
    
    std::vector<DetectedWall> detected_walls = detectWallsWithRANSAC(cloud);
    
    bool has_left = false, has_right = false, has_front = false;
    for(const auto& w : detected_walls) {
        if(w.label == "left") has_left = true;
        if(w.label == "right") has_right = true;
        if(w.label == "front") has_front = true;
    }

    if (!has_left || !has_right || !has_front) {
        ROS_DEBUG_THROTTLE(2.0, "等待检测三面墙... (L:%d, R:%d, F:%d)", has_left, has_right, has_front);
        return false;
    }
    
    // 提取结果到成员变量
    for (const auto& wall : detected_walls) {
        LaserLine line;
        line.label = wall.label;
        line.point_count = wall.point_count;
        line.raw_points = wall.points;
        line.avg_distance = wall.avg_distance; // PCA 精度距离
        line.normal = wall.pca_normal;         // PCA 精度法向量
        line.angle_center = std::atan2(wall.pca_normal.y(), wall.pca_normal.x());
        line.confidence = std::min(1.0f, line.point_count / 100.0f);
        
        if (wall.label == "left") left_line_ = line;
        else if (wall.label == "right") right_line_ = line;
        else if (wall.label == "front") front_line_ = line;
    }
    
    return true;
}

// RANSAC 粗提取 + PCA 精修
std::vector<DetectedWall> LidarCalibration::detectWallsWithRANSAC(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<DetectedWall> walls;
    
    // 角度搜索区域（角度范围较大以防止盲区）
    struct WallRegion { std::string label; float min_angle_deg; float max_angle_deg; };
    std::vector<WallRegion> wall_regions = {
        {"front", -50.0f, 50.0f},
        {"left", 40.0f, 140.0f},
        {"right", -140.0f, -40.0f}
    };
    
    for (const auto& region : wall_regions) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr region_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        float min_rad = toRadians(region.min_angle_deg);
        float max_rad = toRadians(region.max_angle_deg);
        
        // 1. 区域滤波
        for (const auto& point : cloud->points) {
            float r = std::sqrt(point.x*point.x + point.y*point.y);
            if(r < 1.0f || r > 15.0f) continue; // 距离过滤

            float angle = std::atan2(point.y, point.x);
            if (region.label == "right" && angle > 0) angle -= 2.0f * M_PI; // 处理 -PI/+PI 跳变
            
            if (angle >= min_rad && angle <= max_rad) {
                region_cloud->push_back(point);
            }
        }
        
        if (region_cloud->size() < 30) continue;
        
        // 2. RANSAC 拟合直线
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.10); // 10cm 宽阈值，确保能抓到墙
        seg.setMaxIterations(1000);
        seg.setInputCloud(region_cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < 20) continue;
        
        // 3. PCA 精确计算参数
        Eigen::Vector3f pca_result = refineLineParametersPCA(region_cloud, inliers->indices);
        Eigen::Vector3f normal(pca_result.x(), pca_result.y(), 0.0f);
        float distance = pca_result.z();
        
        // 4. 保存内点
        std::vector<pcl::PointXYZ> wall_points;
        for (int idx : inliers->indices) {
            wall_points.push_back(region_cloud->points[idx]);
        }
        
        DetectedWall wall;
        wall.label = region.label;
        wall.avg_distance = distance;
        wall.pca_normal = normal;
        wall.point_count = inliers->indices.size();
        wall.points = wall_points;
        
        walls.push_back(wall);
    }
    return walls;
}

// 计算单帧标定值
void LidarCalibration::calculateCalibrationStep()
{
    if (left_line_.point_count == 0 || right_line_.point_count == 0 || front_line_.point_count == 0) return;

    // --- 1. 计算 Yaw (基于法向量对齐) ---
    // 理论上: Front墙法向量应指向X负轴(180度), Left指向Y负轴(-90度), Right指向Y正轴(90度)
    // Yaw = 理论角度 - 测量角度
    float meas_yaw_front = front_line_.angle_center;
    float meas_yaw_left  = left_line_.angle_center;
    float meas_yaw_right = right_line_.angle_center;

    float yaw_est_front = normalizeAngleRad(M_PI - meas_yaw_front);
    float yaw_est_left  = normalizeAngleRad(-M_PI/2.0f - meas_yaw_left);
    float yaw_est_right = normalizeAngleRad(M_PI/2.0f - meas_yaw_right);

    // 向量加权平均
    float w_f = front_line_.confidence;
    float w_l = left_line_.confidence;
    float w_r = right_line_.confidence;

    float sum_sin = w_f*sin(yaw_est_front) + w_l*sin(yaw_est_left) + w_r*sin(yaw_est_right);
    float sum_cos = w_f*cos(yaw_est_front) + w_l*cos(yaw_est_left) + w_r*cos(yaw_est_right);
    float avg_yaw = std::atan2(sum_sin, sum_cos);

    // --- 2. 计算 Translation (修正Yaw后) ---
    // X轴平移由前墙决定
    float tx = actual_front_dist_ - front_line_.avg_distance;
    
    // Y轴平移由左右墙加权平均决定
    float ty_from_left  = actual_left_dist_ - left_line_.avg_distance;
    float ty_from_right = right_line_.avg_distance - actual_right_dist_;
    float ty = (ty_from_left * w_l + ty_from_right * w_r) / (w_l + w_r);

    // 更新瞬时变换矩阵
    transform_matrix_.setIdentity();
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(avg_yaw, Eigen::Vector3f::UnitZ());
    transform_matrix_.block<3,3>(0,0) = R;
    transform_matrix_(0,3) = tx;
    transform_matrix_(1,3) = ty;
    transform_matrix_(2,3) = manual_lidar_height_;
    
    calibration_done_ = true;
}

// 累积并平滑结果
void LidarCalibration::accumulateAndSmooth()
{
    float current_yaw = std::atan2(transform_matrix_(1,0), transform_matrix_(0,0));
    
    // 检查一致性（用于诊断）
    float ty_left_check  = actual_left_dist_ - left_line_.avg_distance;
    float ty_right_check = right_line_.avg_distance - actual_right_dist_;
    float right_error = std::abs(ty_left_check - ty_right_check);

    // 房间宽度不匹配警告 (仅在误差较大时提示)
    if (right_error > 0.15f) { 
        float config_width = actual_left_dist_ + actual_right_dist_;
        float meas_width = left_line_.avg_distance + right_line_.avg_distance;
        ROS_WARN_THROTTLE(5.0, "宽度不匹配: 预设 %.2fm vs 实测 %.2fm (差: %.2fm). 建议检查配置.", 
            config_width, meas_width, std::abs(config_width - meas_width));
    }

    // 加入队列
    CalibrationResult res;
    res.transform = transform_matrix_;
    res.yaw = current_yaw;
    res.right_error = right_error;
    res.quality = left_line_.confidence + right_line_.confidence + front_line_.confidence;

    calibration_queue_.push_back(res);
    
    // 保持队列长度约 50 帧 (10Hz下约5秒)
    if (calibration_queue_.size() > 50) {
        calibration_queue_.pop_front();
    }
    
    // 计算平滑后的结果
    computeRobustAverage();
}

// 鲁棒平均计算
void LidarCalibration::computeRobustAverage()
{
    if (calibration_queue_.size() < 10) return; // 初始数据积累

    double sum_sin = 0, sum_cos = 0;
    double sum_x = 0, sum_y = 0;
    int count = 0;

    for (const auto& res : calibration_queue_) {
        sum_sin += std::sin(res.yaw);
        sum_cos += std::cos(res.yaw);
        sum_x += res.transform(0, 3);
        sum_y += res.transform(1, 3);
        count++;
    }

    float final_yaw = std::atan2(sum_sin / count, sum_cos / count);
    float final_x = sum_x / count;
    float final_y = sum_y / count;

    // 用平滑后的值覆盖
    transform_matrix_.setIdentity();
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(final_yaw, Eigen::Vector3f::UnitZ());
    transform_matrix_.block<3,3>(0,0) = R;
    transform_matrix_(0,3) = final_x;
    transform_matrix_(1,3) = final_y;
    transform_matrix_(2,3) = manual_lidar_height_;
}

// --- 辅助工具函数 ---

void LidarCalibration::publishTF()
{
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = base_frame_;
    ts.child_frame_id = lidar_frame_;
    
    ts.transform.translation.x = transform_matrix_(0, 3);
    ts.transform.translation.y = transform_matrix_(1, 3);
    ts.transform.translation.z = transform_matrix_(2, 3);
    
    Eigen::Matrix3f rot = transform_matrix_.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rot);
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();
    
    tf_broadcaster_.sendTransform(ts);
}

void LidarCalibration::publishStatus()
{
    nav_msgs::Odometry status;
    status.header.stamp = ros::Time::now();
    status.header.frame_id = base_frame_;
    status.child_frame_id = lidar_frame_;
    
    status.pose.pose.position.x = transform_matrix_(0, 3);
    status.pose.pose.position.y = transform_matrix_(1, 3);
    status.pose.pose.position.z = transform_matrix_(2, 3);
    
    Eigen::Quaternionf q(transform_matrix_.block<3, 3>(0, 0));
    status.pose.pose.orientation.x = q.x();
    status.pose.pose.orientation.y = q.y();
    status.pose.pose.orientation.z = q.z();
    status.pose.pose.orientation.w = q.w();
    
    status_pub_.publish(status);
}

void LidarCalibration::printCalibrationResults()
{
    float x = transform_matrix_(0, 3);
    float y = transform_matrix_(1, 3);
    float yaw = std::atan2(transform_matrix_(1, 0), transform_matrix_(0, 0));
    
    ROS_INFO("================ 标定输出 (稳定帧数: %lu) ================", calibration_queue_.size());
    
    // 不再调用 toDegrees(yaw)，单位显示改为 rad
    ROS_INFO("Extrinsic TF: x=%.4f m, y=%.4f m, yaw=%.4f rad", x, y, yaw);
    
    // 实测房间宽度诊断 (用于校正参数)
    float meas_width = left_line_.avg_distance + right_line_.avg_distance;
    ROS_INFO("环境诊断: 实测房间宽度 %.3fm (Left:%.3f + Right:%.3f)", meas_width, left_line_.avg_distance, right_line_.avg_distance);
}

void LidarCalibration::publishWallPoints() {
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    auto add_pts = [&](const LaserLine& line, int r, int g, int b) {
        for(auto& p : line.raw_points) {
            pcl::PointXYZRGB cp; cp.x=p.x; cp.y=p.y; cp.z=p.z; cp.r=r; cp.g=g; cp.b=b;
            colored_cloud.push_back(cp);
        }
    };
    add_pts(left_line_, 255, 0, 0);   // 红: 左
    add_pts(right_line_, 0, 255, 0);  // 绿: 右
    add_pts(front_line_, 0, 0, 255);  // 蓝: 前
    
    if(!colored_cloud.empty()) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(colored_cloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = lidar_frame_;
        wall_points_pub_.publish(msg);
    }
}

void LidarCalibration::publishCalibratedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed, transform_matrix_);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*transformed, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = base_frame_;
    calibrated_pub_.publish(msg);
}

float LidarCalibration::normalizeAngleRad(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float LidarCalibration::toRadians(float degrees) { return degrees * M_PI / 180.0f; }
float LidarCalibration::toDegrees(float radians) { return radians * 180.0f / M_PI; }

void LidarCalibration::run() {
    ros::spin();
}




