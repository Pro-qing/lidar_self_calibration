<!-- 
策略：
先定义基准墙面，可以通过三面墙构成一个空间。然后通过雷达获取的数据，拟合出三面墙，然后通过对比两个空间，可以得出x,y,yaw,roll,pitch的值，z需要手动测量。

getq@getq:~/laser_self_calibration_workspace$ rosnode info /lidar_self_calibration 
--------------------------------------------------------------------------------
Node [/lidar_self_calibration]
Publications: 
 * /lidar_self_calibration/calibrated_points [sensor_msgs/PointCloud2]
 * /lidar_self_calibration/calibration_status [nav_msgs/Odometry]
 * /lidar_self_calibration/filtered_points [sensor_msgs/PointCloud2]
 * /lidar_self_calibration/wall_points [sensor_msgs/PointCloud2]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

Subscriptions: 
 * /points_16 [sensor_msgs/PointCloud2]

Services: 
 * /lidar_self_calibration/get_loggers
 * /lidar_self_calibration/set_logger_level

-->