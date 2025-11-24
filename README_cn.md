# TurboPi ROS2

[English](https://github.com/Hiwonder/TurboPi/blob/ros2/README.md) | 中文

<p align="center">
  <img src="sources/images/turbopi.png" alt="TurboPi Logo" width="600"/>
</p>

基于ROS2的TurboPi智能机器人控制系统，集成计算机视觉、自动避障、颜色追踪、人脸识别等AI功能的分布式机器人应用。

## 产品介绍

TurboPi是一款基于树莓派开发，专为初学者设计的开源AI视觉小车。它采用麦克纳姆轮底盘，配备2自由度高清广角摄像头，融合OpenCV视觉库与YOLOV5深度学习框架，可实现多模态感知与运动控制实验，能够高效完成颜色识别、目标追踪、无人驾驶、人脸与手势识别等AI功能！

TurboPi使用ROS2机器人操作系统，并部署了多模态AI大模型，搭配AI语音交互盒，可以使其获得类人认知与决策能力。通过多模态感知与决策，轻松完成自然语言交互、自主导航、智能管家与动态避障等高阶具身智能应用能力，全面满足复杂应用场景需求！

## 主要功能

### AI视觉功能
- **颜色追踪** - 基于ROS2节点的颜色识别与追踪
- **人脸追踪** - MediaPipe集成的实时人脸检测与追踪
- **手势识别** - 手势命令识别和机器人控制
- **巡线功能** - 自动巡线导航节点
- **二维码识别** - QuickMark二维码检测与解析

### 智能控制
- **自动避障** - 基于超声波传感器的分布式避障系统
- **麦克纳姆轮控制** - ROS2运动控制节点
- **远程控制** - 支持ROS2话题和服务的远程操控
- **云台控制** - 2自由度摄像头云台控制节点

### ROS2架构特性
- **分布式节点** - 模块化的ROS2节点架构
- **话题通信** - 标准ROS2消息传递
- **服务调用** - 同步和异步服务接口
- **参数服务器** - 动态参数配置
- **Launch文件** - 系统启动和配置管理

## 硬件配置

- **处理器**: 树莓派5B
- **移动系统**: 麦克纳姆轮全向移动底盘
- **视觉系统**: USB摄像头 + 2自由度云台
- **传感器**: 超声波距离传感器、四路巡线传感器
- **执行器**: PWM舵机、直流电机
- **指示器**: RGB LED灯、蜂鸣器

## ROS2包结构

```
turbopi_ros2/
├── src/
│   ├── app/                        # 应用层功能包
│   │   ├── app/
│   │   │   ├── avoidance_node.py   # 避障节点
│   │   │   ├── gesture_control_node.py  # 手势控制节点
│   │   │   ├── hand_trajectory_node.py  # 手部轨迹节点
│   │   │   ├── line_following.py   # 巡线功能
│   │   │   ├── tracking.py         # 目标追踪
│   │   │   └── qrcode.py          # 二维码识别
│   │   └── launch/                 # 启动文件
│   ├── bringup/                   # 系统启动包
│   ├── driver/                    # 硬件驱动层
│   │   ├── controller/            # 控制器驱动
│   │   ├── ros_robot_controller/  # 机器人控制器
│   │   ├── ros_robot_controller_msgs/  # 控制器消息
│   │   └── sdk/                   # 硬件SDK
│   ├── example/                   # 示例代码
│   ├── interfaces/                # 接口定义
│   ├── large_models/              # 大模型集成
│   └── large_models_msgs/         # 大模型消息
```

## 官方资源

### Hiwonder官方
- **官方网站**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **产品页面**: [https://www.hiwonder.com/products/turbopi](https://www.hiwonder.com/products/turbopi)
- **官方文档**: [https://docs.hiwonder.com/projects/TurboPi/en/advanced/](https://docs.hiwonder.com/projects/TurboPi/en/advanced/)
- **技术支持**: support@hiwonder.com

### 相关技术
- [ROS2官方文档](https://docs.ros.org/en/humble/) - ROS2开发文档
- [OpenCV](https://opencv.org/) - 计算机视觉库
- [MediaPipe](https://mediapipe.dev/) - 机器学习框架

## 版本信息

- **当前版本**: 20250815
- **ROS2版本**: Humble/Galactic/Foxy
- **支持平台**: 树莓派5B

---

**注**: 本ROS2版本为TurboPi的分布式重构版本，提供更好的模块化和可扩展性。详细使用教程请参考[官方文档](https://docs.hiwonder.com/projects/TurboPi/en/advanced/)。