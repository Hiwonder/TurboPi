#!/usr/bin/python3
# coding=utf8

import cv2
import math
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ros_robot_controller_msgs.msg import BuzzerState, RGBStates
from cv_bridge import CvBridge
import sdk.yaml_handle as yaml_handle
import sdk.Misc as Misc


class ColorWarningNode(Node):
    def __init__(self):
        super().__init__('color_warning_node')

        # 加载配置文件
        self.load_config()

        # 初始化 OpenCV Bridge
        self.bridge = CvBridge()

        # 设置订阅者
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # 设置蜂鸣器发布器
        self.buzzer_publisher = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        
        # 设置RGB灯控制发布器
        self.rgb_pub = self.create_publisher(RGBStates, '/ros_robot_controller/set_rgb', 1)

        # 初始化状态变量
        self.detect_color = 'None'
        self.draw_color = (0, 0, 0)
        self.di_once = True

        # 启动蜂鸣器线程
        th = threading.Thread(target=self.buzzer)
        th.daemon = True
        th.start()

    def load_config(self):
        """加载颜色检测配置"""
        global lab_data
        lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

    def buzzer(self):
        """控制蜂鸣器的响铃"""
        while rclpy.ok():
            if self.detect_color == 'red' and self.di_once:
                # 蜂鸣器开始响
                self.publish_buzzer_state(True)
                self.di_once = False
            elif self.detect_color != 'red' and not self.di_once:
                # 蜂鸣器停止
                self.publish_buzzer_state(False)
                self.di_once = True
            time.sleep(0.01)

    def publish_buzzer_state(self, state: bool):
        """发布蜂鸣器状态消息"""
        buzzer_msg = BuzzerState()
        if state:
            buzzer_msg.freq = 1900
            buzzer_msg.on_time = 0.2
            buzzer_msg.off_time = 0.01
            buzzer_msg.repeat = 1
        else:
            buzzer_msg.freq = 0
            buzzer_msg.on_time = 0.0
            buzzer_msg.off_time = 0.0
            buzzer_msg.repeat = 0
        self.buzzer_publisher.publish(buzzer_msg)

    def image_callback(self, msg):
        """处理从摄像头获取的图像"""
        # 将ROS图像消息转换为OpenCV图像
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 处理图像并检测颜色
        frame = self.run(img)
        
        # 显示处理后的图像
        cv2.imshow('Frame', frame)
        key = cv2.waitKey(1)
        if key == 27:  # 按ESC退出
            self.destroy_node()
            cv2.destroyAllWindows()

    def run(self, img):
        """颜色检测处理"""
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        # 图像缩放与高斯模糊处理
        frame_resize = cv2.resize(img_copy, (320, 240), interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        max_area = 0
        color_area_max = None    
        areaMaxContour_max = 0
        
        # 遍历配置文件中的颜色范围进行颜色检测
        for color_name, color_data in lab_data.items():
            if color_name not in ['black', 'white']:
                frame_mask = cv2.inRange(frame_lab,
                                        (color_data['min'][0], color_data['min'][1], color_data['min'][2]),
                                        (color_data['max'][0], color_data['max'][1], color_data['max'][2]))
                # 图像膨胀与腐蚀操作以减少噪音
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.get_area_max_contour(contours)
                
                if areaMaxContour is not None:  # 确保 areaMaxContour 存在
                    if area_max > max_area:  # 如果当前区域更大，则更新
                        max_area = area_max
                        color_area_max = color_name
                        areaMaxContour_max = areaMaxContour

        # 如果检测到区域较大，则进行标记
        if max_area > 200:
            ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)
            centerX = int(Misc.map(centerX, 0, 320, 0, img_w))
            centerY = int(Misc.map(centerY, 0, 240, 0, img_h))
            radius = int(Misc.map(radius, 0, 320, 0, img_w))
            cv2.circle(img, (centerX, centerY), radius, self.get_color(color_area_max), 2)
            
            # 更新颜色状态
            self.detect_color = color_area_max
            self.draw_color = self.get_color(color_area_max)
        else:
            self.detect_color = 'None'
            self.draw_color = (0, 0, 0)

        # 显示颜色名称
        cv2.putText(img, f"Color: {self.detect_color}", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)
        return img


    def get_area_max_contour(self, contours):
        """获取最大轮廓"""
        max_area = 0
        max_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area and area > 50:
                max_area = area
                max_contour = c
        return max_contour, max_area

    def get_color(self, color_name):
        """返回颜色的RGB值"""
        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'black': (0, 0, 0),
            'purple':(255, 255, 126),
        }
        return color_map.get(color_name, (0, 0, 0))


def main():
    rclpy.init()
    color_warning_node = ColorWarningNode()
    rclpy.spin(color_warning_node)

    # 关闭节点
    color_warning_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
