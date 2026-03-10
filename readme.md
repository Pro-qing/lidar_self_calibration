# Lidar Self-Calibration (YAML-CPP Edition)

![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg) ![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg) ![Language](https://img.shields.io/badge/Language-C++-green.svg)

这是一个基于 **ROS Noetic** 开发的激光雷达外参自动标定工具。它通过识别室内（或实验室）环境中的三面垂直墙面（前、左、右），结合 **RANSAC 算法**与 **PCA (主成分分析)** 提取高精度特征，从而计算出雷达相对于车体中心 (`base_link`) 的位姿修正值，并利用 `yaml-cpp` 实现结果的自动持久化。

## 🚀 功能特性

* **高精度检测**：结合 RANSAC 粗提取与 PCA 精修，获取亚厘米级的墙面法向量与距离。
* **鲁棒性平滑**：采用滑动窗口（默认 50 帧）对标定结果进行均值平滑，消除动态噪声。
* **自动保存**：当标定数据达到稳定后，每隔 100 帧自动将外参写入指定的 `.yaml` 文件。
* **实时可视化**：通过 RViz 实时查看提取的墙面点（颜色区分）及校准后的点云对齐效果。

## 🛠 环境要求

* **操作系统**: Ubuntu 20.04 (推荐)
* **ROS 版本**: Noetic
* **依赖库**: 
    * `PCL` (Point Cloud Library)
    * `Eigen3`
    * `yaml-cpp` (系统安装：`sudo apt-get install libyaml-cpp-dev`)

## 📂 项目结构

```text
lidar_self_calibration/
├── include/
│   └── lidar_self_calibration/
│       └── lidar_self_calibration.hpp  # 核心逻辑类定义
├── src/
│   ├── lidar_self_calibration.cpp      # 标定算法与 YAML 读写实现
│   └── main.cpp                        # 节点入口
├── launch/
│   └── lidar_self_calibration.launch   # 启动文件与参数配置
├── param/
│   └── lidar_calibration.yaml          # 自动生成的外参结果文件
├── CMakeLists.txt
└── package.xml
```

## 🔨 安装与编译

    进入你的工作空间 src 目录：
```Bash

cd ~/catkin_ws/src
```
克隆仓库：
```Bash

git clone [https://github.com/Pro-qing/lidar_self_calibration.git](https://github.com/Pro-qing/lidar_self_calibration.git)
```
编译：
```Bash

    cd ..
    catkin_make
    source devel/setup.bash
```
## ⚙️ 配置说明

在运行前，请在 launch/lidar_self_calibration.launch 中根据实际环境测量并填写以下关键参数：
```bash
参数名	类型	说明
actual_left_distance	double	车体中心到左侧墙面的真实距离 (m)
actual_right_distance	double	车体中心到右侧墙面的真实距离 (m)
actual_front_distance	double	车体中心到前方墙面的真实距离 (m)
save_path	string	YAML 结果文件的绝对路径
```
## 🏁 运行指南

启动标定节点：
```Bash

roslaunch lidar_self_calibration lidar_self_calibration.launch
```
标定流程：

    将机器人置于 U 型或矩形车间环境中心，保持静止。

    观察终端输出，当显示 标定输出 (稳定帧数: 50) 时，表示数据已收敛。

    程序会自动在 save_path 路径下生成/更新 YAML 文件。

## 📄 标定结果示例

生成的 lidar_calibration.yaml 内容格式如下：
YAML

lidar_calibration:
  x: 0.1234
  y: -0.0567
  z: 1.0
  yaw: 0.0152
  pitch: 0.0
  roll: 0.0
  frame_id: "base_link"
  child_frame_id: "velodyne"

## Maintainer: Pro-qing