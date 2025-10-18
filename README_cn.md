# TurboPi

[English](https://github.com/Hiwonder/TurboPi/blob/main/README.md) | 中文

<p align="center">
  <img src="./sources/images/image.png" alt="TurboPi Logo" width="400"/>
</p>

基于树莓派的智能机器人控制系统，集成计算机视觉、自动避障、颜色追踪、人脸识别等AI功能。

## 产品介绍

TurboPi是一款基于树莓派开发，专为初学者设计的开源AI视觉小车。它采用麦克纳姆轮底盘，配备2自由度高清广角摄像头，融合OpenCV视觉库与YOLOV5深度学习框架，可实现多模态感知与运动控制实验，能够高效完成颜色识别、目标追踪、无人驾驶、人脸与手势识别等AI功能！

## 主要功能

### AI视觉功能

- **颜色追踪** - 识别并追踪指定颜色物体
- **人脸追踪** - 基于MediaPipe的实时人脸检测与追踪
- **手势识别** - 手势命令识别和响应
- **巡线功能** - 自动巡线行走
- **二维码识别** - QuickMark二维码检测与解析

### 智能控制

- **自动避障** - 基于超声波传感器的智能避障
- **麦克纳姆轮控制** - 全向移动控制
- **远程控制** - 支持APP和网络远程操控
- **云台控制** - 2自由度摄像头云台

### 编程接口

- **Python编程** - 完整的Python SDK
- **RPC接口** - JSON-RPC远程调用
- **视频流** - 实时MJPG视频流传输

## 硬件配置

- **处理器**: 树莓派4B或5B
- **移动系统**: 麦克纳姆轮全向移动底盘
- **视觉系统**: USB摄像头 + 2自由度云台
- **传感器**: 超声波距离传感器、四路巡线传感器
- **执行器**: PWM舵机、直流电机
- **指示器**: RGB LED灯、蜂鸣器

## 项目结构

```
turbopi/
├── TurboPi.py                  # 主程序入口
├── Camera.py                   # 摄像头控制
├── RPCServer.py               # RPC服务器
├── MjpgServer.py              # 视频流服务器
├── Functions/                 # 功能模块
│   ├── ColorTracking.py       # 颜色追踪
│   ├── FaceTracking.py        # 人脸追踪
│   ├── GestureRecognition.py  # 手势识别
│   ├── Avoidance.py           # 避障功能
│   ├── LineFollower.py        # 巡线功能
│   └── QuickMark.py           # 二维码识别
├── HiwonderSDK/               # 硬件控制SDK
├── CameraCalibration/         # 摄像头标定工具
├── lab_config.yaml            # 颜色识别配置
└── servo_config.yaml          # 舵机配置
```

## 官方资源

### Hiwonder官方

- **官方网站**: [https://www.hiwonder.net/](https://www.hiwonder.net/)
- **产品页面**: [https://www.hiwonder.com/products/turbopi](https://www.hiwonder.com/products/turbopi)
- **官方文档**: [https://docs.hiwonder.com/projects/TurboPi/en/standard/](https://docs.hiwonder.com/projects/TurboPi/en/standard/)
- **技术支持**: support@hiwonder.com

### 相关技术

- [OpenCV](https://opencv.org/) - 计算机视觉库
- [MediaPipe](https://mediapipe.dev/) - 机器学习框架

## 版本信息

- **当前版本**: v1.0.0 (2024-03-14)
- **支持平台**: 树莓派4B或5B

---

**注**: 所有程序已预装在TurboPi机器人系统中，可直接运行。详细使用教程请参考[官方文档](https://docs.hiwonder.com/projects/TurboPi/en/standard/)。