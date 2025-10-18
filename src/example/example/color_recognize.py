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
from ros_robot_controller_msgs.msg import BuzzerState, RGBStates, SetPWMServoState, PWMServoState, RGBState  # 导入 RGBState
from cv_bridge import CvBridge
import sdk.yaml_handle as yaml_handle
import sdk.Misc as Misc


class ColorWarningNode(Node):
    def __init__(self):
        super().__init__('color_warning_node')

        self.load_config()

        self.bridge = CvBridge()

        # 设置订阅者
        self.image_subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            1
        )

        # 设置蜂鸣器发布器
        self.buzzer_publisher = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)

        self.rgb_pub = self.create_publisher(RGBStates, '/ros_robot_controller/set_rgb', 1)

        # 设置舵机控制发布器
        self.pwm_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 10)

        # 初始化状态变量
        self.detect_color = 'None'
        self.draw_color = (0, 0, 0)
        self.di_once = True
        self.servo_speed = 0.2
        # 记录上一次检测到的颜色
        self.previous_color = 'None'

        # 锁，用于线程同步
        self.lock = threading.Lock()
        self.current_image = None
        self.running = True # 用于控制主循环

        th = threading.Thread(target=self.buzzer)
        th.daemon = True
        th.start()

        # 启动图像处理线程
        self.image_processing_thread = threading.Thread(target=self.process_image)
        self.image_processing_thread.daemon = True
        self.image_processing_thread.start()

    def load_config(self):
        """加载颜色检测配置"""
        global lab_data
        lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

    def buzzer(self):
        """控制蜂鸣器的响铃"""
        while rclpy.ok() and self.running: 
            with self.lock:
                if self.detect_color == 'red' and self.di_once:
                    self.publish_buzzer_state(True)
                    self.di_once = False
                elif self.detect_color != 'red' and not self.di_once:
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
        with self.lock:
            self.current_image = img

    def process_image(self):
        """图像处理线程"""
        while rclpy.ok() and self.running: 
            img = None  
            with self.lock:
                if self.current_image is not None:
                    img = self.current_image.copy()  
                    self.current_image = None  
            if img is not None:
                frame = self.run(img)
                cv2.imshow('Frame', frame)
                key = cv2.waitKey(1)
                if key == 27:  # 按ESC退出
                    self.running = False
                    cv2.destroyAllWindows()
                    break
            else:
                time.sleep(0.01) 

    def run(self, img):
        """颜色检测处理"""
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

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
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.get_area_max_contour(contours)

                if areaMaxContour is not None:  # 确保 areaMaxContour 存在
                    if area_max > max_area:
                        max_area = area_max
                        color_area_max = color_name
                        areaMaxContour_max = areaMaxContour

        with self.lock:  # 在访问共享资源时加锁
            if max_area > 2000:
                ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)
                centerX = int(Misc.map(centerX, 0, 320, 0, img_w))
                centerY = int(Misc.map(centerY, 0, 240, 0, img_h))
                radius = int(Misc.map(radius, 0, 320, 0, img_w))
                cv2.circle(img, (centerX, centerY), radius, self.get_color(color_area_max), 2)

                self.detect_color = color_area_max
                self.draw_color = self.get_color(color_area_max)

                if self.detect_color != self.previous_color:
                    if self.detect_color == 'red':
                        self.nod_head()
                    else:
                        self.shake_head()

                    self.previous_color = self.detect_color

            # 根据检测到的颜色设置 RGB 灯 (始终设置)
                self.set_rgb_color(self.detect_color)  # 始终调用

            else:
                self.detect_color = 'None'
                self.draw_color = (0, 0, 0)
                # 如果没有检测到颜色，将上次检测到的颜色重置为 None
                if self.previous_color != 'None':
                    self.previous_color = 'None'

            # 没有检测到颜色，关闭 RGB 灯
                self.set_rgb_color('None') # 始终调用

        # 显示颜色名称
        cv2.putText(img, f"Color: {self.detect_color}", (10, img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)
        return img

    def nod_head(self):
        """点头动作"""
        self.pwm_controller([1, 1800])
        time.sleep(0.3)
        self.pwm_controller([1, 1200])
        time.sleep(0.3)
        self.pwm_controller([1, 1500])
        time.sleep(0.3)
    def shake_head(self):
        """摇头动作"""
        self.pwm_controller([2, 1200])
        time.sleep(0.3)
        self.pwm_controller([2, 1800])
        time.sleep(0.3)
        self.pwm_controller([2, 1500])
        time.sleep(0.3)

    def reset_head(self):
        """将舵机恢复到中间位置"""
        self.pwm_controller([self.head_nod_servo_id, self.head_center_position])
        self.pwm_controller([self.head_shake_servo_id, self.head_center_position])

    def pwm_controller(self, position_data):
        """控制舵机"""
        msg = SetPWMServoState()
        msg.duration = self.servo_speed  # 设置舵机移动的持续时间
        pos = PWMServoState()
        pos.id = [position_data[0]]
        pos.position = [int(position_data[1])]
        msg.state = [pos]
        self.pwm_pub.publish(msg)

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
            'purple': (255, 255, 126), 
        }
        return color_map.get(color_name, (0, 0, 0))

    def set_rgb_color(self, color_name):
        """设置 RGB 灯的颜色"""
        rgb_msg = RGBStates()
        rgb_msg.states = []  # 初始化状态列表

        if color_name == 'red':
            rgb_msg.states.append(RGBState(index=1, red=255, green=0, blue=0))  # 设置 LED 1 为红色
            rgb_msg.states.append(RGBState(index=2, red=255, green=0, blue=0))  # 设置 LED 2 为红色
        elif color_name == 'green':
            rgb_msg.states.append(RGBState(index=1, red=0, green=255, blue=0))  # 设置 LED 1 为绿色
            rgb_msg.states.append(RGBState(index=2, red=0, green=255, blue=0))  # 设置 LED 2 为绿色
        elif color_name == 'blue':
            rgb_msg.states.append(RGBState(index=1, red=0, green=0, blue=255))  # 设置 LED 1 为蓝色
            rgb_msg.states.append(RGBState(index=2, red=0, green=0, blue=255))  # 设置 LED 2 为蓝色
        elif color_name == 'yellow':
            rgb_msg.states.append(RGBState(index=1, red=255, green=255, blue=0))  # 设置 LED 1 为黄色
            rgb_msg.states.append(RGBState(index=2, red=255, green=255, blue=0))  # 设置 LED 2 为黄色
        elif color_name == 'purple':
            rgb_msg.states.append(RGBState(index=1, red=192, green=0, blue=192))  
            rgb_msg.states.append(RGBState(index=2, red=192, green=0, blue=192))
        else:  # 'None' 或者其他未定义的颜色
            rgb_msg.states.append(RGBState(index=1, red=0, green=0, blue=0))  # 关闭 LED 1
            rgb_msg.states.append(RGBState(index=2, red=0, green=0, blue=0))  # 关闭 LED 2

        self.rgb_pub.publish(rgb_msg)


def main():
    rclpy.init()
    color_warning_node = ColorWarningNode()
    try:
        rclpy.spin(color_warning_node)
    finally:
        # 退出前进行清理
        color_warning_node.running = False  # 设置标志以停止线程
        color_warning_node.destroy_node()  # 确保销毁节点
        rclpy.shutdown()
        cv2.destroyAllWindows() # 确保关闭 OpenCV 窗口


if __name__ == '__main__':
    main()
