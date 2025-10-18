#!/usr/bin/python3
# coding=utf8

import cv2
import math
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sdk.yaml_handle as yaml_handle
import sdk.Misc as Misc
import numpy as np  # 确保导入 numpy


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

        # 初始化状态变量
        self.detect_color = 'None'
        self.draw_color = (0, 0, 0)
        self.color_radius = 0  # 初始化颜色半径
        self.color_center_x = -1  # 初始化颜色中心X坐标
        self.color_center_y = -1  # 初始化颜色中心Y坐标

        # 锁，用于线程同步
        self.lock = threading.Lock()
        self.current_image = None
        self.running = True  # 用于控制主循环
        self.__isRunning = True  # 添加 __isRunning 变量

        # 从配置文件中加载颜色数据
        self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
        self.size = (320, 240)  # 指定图像缩放尺寸
        self.range_rgb = {'red': (0, 0, 255), 'green': (0, 255, 0),
                          'blue': (255, 0, 0), 'yellow': (0, 255, 255),
                          'purple': (255, 0, 255), 'cyan': (255, 255, 0)} # 颜色字典，添加更多颜色

        # 启动图像处理线程
        self.image_processing_thread = threading.Thread(target=self.process_image)
        self.image_processing_thread.daemon = True
        self.image_processing_thread.start()

    def load_config(self):
        """加载颜色检测配置"""
        global lab_data
        lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

    def image_callback(self, msg):
        """处理从摄像头获取的图像"""
        # 将ROS图像消息转换为OpenCV图像
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.current_image = img

    def process_image(self):
        """图像处理线程"""
        while rclpy.ok() and self.running:  # 使用 self.running 来控制线程
            img = None  # 初始化 img 变量
            with self.lock:
                if self.current_image is not None:
                    img = self.current_image.copy()  # 创建图像的副本进行处理
                    self.current_image = None  # 清空当前图像，以便接收新的图像

            if img is not None:
                frame = self.run(img)
                cv2.imshow('Frame', frame)
                key = cv2.waitKey(1)
                if key == 27:  # 按ESC退出
                    self.running = False
                    cv2.destroyAllWindows()
                    break
            else:
                time.sleep(0.01)  # 如果没有图像，则稍作等待

    def run(self, img):
        """颜色检测处理"""
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        if not self.__isRunning:   # 检测是否开启玩法，没有开启则返回原图像(check if the program is enabled, return the original image if not enabled)
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to the LAB space)

        max_area = 0
        areaMaxContour = None #fixed

        detected_color = 'None'
        # 遍历所有可能的颜色
        for color_name in self.lab_data.keys():
            if color_name in ['black', 'white']:  # 排除黑色和白色
                continue

            frame_mask = cv2.inRange(frame_lab,
                                         (self.lab_data[color_name]['min'][0],
                                          self.lab_data[color_name]['min'][1],
                                          self.lab_data[color_name]['min'][2]),
                                         (self.lab_data[color_name]['max'][0],
                                          self.lab_data[color_name]['max'][1],
                                          self.lab_data[color_name]['max'][2]))
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算(opening operation)
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算(closing operation)
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
            contour, area = self.getAreaMaxContour(contours)  # 找出最大轮廓(find the maximal contour)

            if area > max_area:  # 选择面积最大的轮廓
                max_area = area
                areaMaxContour = contour
                detected_color = color_name #added

        if max_area > 2000 and areaMaxContour is not None:  # 有找到最大面积(the maximal area is found)
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)  # 获取最小外接圆(get the minimum circumcircle)
            self.color_radius = int(Misc.map(radius, 0, self.size[0], 0, img_w))
            self.color_center_x = int(Misc.map(center_x, 0, self.size[0], 0, img_w))
            self.color_center_y = int(Misc.map(center_y, 0, self.size[1], 0, img_h))

            if self.color_radius > 300:  # 限制半径大小
                self.color_radius = 0
                self.color_center_x = -1
                self.color_center_y = -1
                return img

            # 根据检测到的颜色绘制圆圈，如果没有找到颜色则默认为红色
            color = self.range_rgb.get(detected_color, (0, 0, 255)) #fixed error

            cv2.circle(img, (self.color_center_x, self.color_center_y), self.color_radius, color, 2)
            cv2.putText(img, f"X: {self.color_center_x}, Y: {self.color_center_y}",
                        (self.color_center_x + self.color_radius, self.color_center_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.putText(img, f"Color: {detected_color}", (10, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)  # 显示颜色名称
            
        else:
            self.color_radius = 0
            self.color_center_x = -1
            self.color_center_y = -1

        return img

    def getAreaMaxContour(self, contours):
        """获取最大轮廓"""
        max_area = 0
        max_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area:
                max_area = area
                max_contour = c
        return max_contour, max_area


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
