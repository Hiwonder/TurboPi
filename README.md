# TurboPi ROS2

English | [中文](https://github.com/Hiwonder/TurboPi/blob/ros2/README_cn.md)

<p align="center">
  <img src="sources/images/turbopi.png" alt="TurboPi Logo" width="600"/>
</p>

## Product Overview

TurboPi is an open-source AI vision vehicle based on Raspberry Pi, specifically designed for beginners. It features a Mecanum wheel chassis and is equipped with a 2-DOF high-definition wide-angle camera. By integrating the OpenCV vision library with the YOLOV5 deep learning framework, it enables multimodal perception and motion control experiments, efficiently completing AI functions such as color recognition, object tracking, autonomous driving, face and gesture recognition!

TurboPi uses the ROS2 robot operating system and deploys multimodal AI large models, paired with an AI voice interaction box, enabling it to acquire human-like cognitive and decision-making capabilities. Through multimodal perception and decision-making, it can easily complete advanced embodied intelligence applications such as natural language interaction, autonomous navigation, smart butler services, and dynamic obstacle avoidance, fully meeting the demands of complex application scenarios!

## Key Features

### AI Vision Functions
- **Color Tracking** - ROS2 node-based color recognition and tracking
- **Face Tracking** - Real-time face detection and tracking with MediaPipe integration
- **Gesture Recognition** - Gesture command recognition and robot control
- **Line Following** - Automatic line following navigation node
- **QR Code Recognition** - QuickMark QR code detection and parsing

### Intelligent Control
- **Obstacle Avoidance** - Distributed obstacle avoidance system based on ultrasonic sensors
- **Mecanum Wheel Control** - ROS2 motion control node
- **Remote Control** - Remote control supporting ROS2 topics and services
- **Pan-Tilt Control** - 2-DOF camera pan-tilt control node

### ROS2 Architecture Features
- **Distributed Nodes** - Modular ROS2 node architecture
- **Topic Communication** - Standard ROS2 message passing
- **Service Calls** - Synchronous and asynchronous service interfaces
- **Parameter Server** - Dynamic parameter configuration
- **Launch Files** - System startup and configuration management

## Hardware Configuration

- **Processor**: Raspberry Pi 5B
- **Mobility System**: Mecanum wheel omnidirectional mobile chassis
- **Vision System**: USB camera + 2-DOF pan-tilt
- **Sensors**: Ultrasonic distance sensor, 4-channel line following sensor
- **Actuators**: PWM servo, DC motor
- **Indicators**: RGB LED, buzzer

## ROS2 Package Structure

```
turbopi_ros2/
├── src/
│   ├── app/                        # Application layer packages
│   │   ├── app/
│   │   │   ├── avoidance_node.py   # Obstacle avoidance node
│   │   │   ├── gesture_control_node.py  # Gesture control node
│   │   │   ├── hand_trajectory_node.py  # Hand trajectory node
│   │   │   ├── line_following.py   # Line following
│   │   │   ├── tracking.py         # Object tracking
│   │   │   └── qrcode.py          # QR code recognition
│   │   └── launch/                 # Launch files
│   ├── bringup/                   # System bringup package
│   ├── driver/                    # Hardware driver layer
│   │   ├── controller/            # Controller drivers
│   │   ├── ros_robot_controller/  # Robot controller
│   │   ├── ros_robot_controller_msgs/  # Controller messages
│   │   └── sdk/                   # Hardware SDK
│   ├── example/                   # Example code
│   ├── interfaces/                # Interface definitions
│   ├── large_models/              # Large model integration
│   └── large_models_msgs/         # Large model messages
```

## Official Resources

### Official Hiwonder
- **Official Website**: [https://www.hiwonder.net/](https://www.hiwonder.net/)
- **Product Page**: [https://www.hiwonder.com/products/turbopi](https://www.hiwonder.com/products/turbopi)
- **Official Documentation**: [https://docs.hiwonder.com/projects/TurboPi/en/advanced/](https://docs.hiwonder.com/projects/TurboPi/en/advanced/)
- **Technical Support**: support@hiwonder.com

### Related Technologies
- [ROS2 Official Documentation](https://docs.ros.org/en/humble/) - ROS2 Development Documentation
- [OpenCV](https://opencv.org/) - Computer Vision Library
- [MediaPipe](https://mediapipe.dev/) - Machine Learning Framework

## Version Information

- **Current Version**: 20250815
- **ROS2 Version**: Humble/Galactic/Foxy
- **Supported Platform**: Raspberry Pi 5B

---

**Note**: This ROS2 version is a distributed refactored version of TurboPi, providing better modularity and scalability. For detailed tutorials, please refer to the [Official Documentation](https://docs.hiwonder.com/projects/TurboPi/en/advanced/).