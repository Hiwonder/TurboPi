import sys
import time
import math
import signal
import threading
import numpy as np
import cv2
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.service import Service
from example_interfaces.srv import SetBool
import sdk.FourInfrared as infrared

from ros_robot_controller_msgs.msg import RGBStates, RGBState, BuzzerState

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # 发布者初始化
        self.mecanum_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.color_pub = self.create_publisher(String, '/detected_color', 10)
        self.rgb_pub = self.create_publisher(RGBStates, '/ros_robot_controller/set_rgb', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

        # 定时器调用主控制循环，周期0.1秒
        self.timer = self.create_timer(0.1, self.main_control_loop)
        self.bridge = CvBridge()

        # 状态变量初始化
        self.detect_color = 'None'
        self.color_list = []
        self.car_stop = False
        self.isRunning = False
        self.size = (640, 480)
        self.target_color = ('red', 'green', 'yellow')  # 目标颜色序列

        self.buzzer_played = False  # 蜂鸣器响过标志，防止重复响
        self.line_following_enabled = True  # 巡线使能

        # 加载颜色阈值配置
        self.load_config()

        # 启动和停止游戏服务
        self.start_service = self.create_service(SetBool, 'start_game', self.start_game_callback)
        self.stop_service = self.create_service(SetBool, 'stop_game', self.stop_game_callback)

        # 订阅图像话题
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # 初始化红外寻线传感器
        self.line = infrared.FourInfrared()

        # 检查红外传感器状态
        self.check_infrared_sensors()

    def check_infrared_sensors(self):
        try:
            data = self.line.readData()
            self.get_logger().info(f"Infrared sensor data: {data}")
        except Exception as e:
            self.get_logger().error(f"Error reading infrared data: {e}")

    def load_config(self):
        try:
            with open('/home/ubuntu/software/lab_tool/lab_config.yaml', 'r') as f:
                self.lab_data = yaml.safe_load(f)
            self.get_logger().info("Loaded lab_config.yaml successfully")
        except FileNotFoundError:
            self.get_logger().error("lab_config.yaml not found. Please check the path.")
            rclpy.shutdown()
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing lab_config.yaml: {e}")
            rclpy.shutdown()

    def start_game_callback(self, request, response):
        self.isRunning = True
        self.buzzer_played = False
        self.get_logger().info("游戏开始")
        response.success = True
        response.message = "游戏已开始"
        return response

    def stop_game_callback(self, request, response):
        self.isRunning = False
        self.stop_robot()
        self.set_rgb("None")
        self.get_logger().info("游戏停止")
        response.success = True
        response.message = "游戏已停止"
        return response

    def main_control_loop(self):
        if self.isRunning:
            # 根据识别到的颜色切换行为
            if self.detect_color == 'red':
                # 红灯：停止巡线，蜂鸣器响一次，红色灯光，停止机器人
                self.line_following_enabled = False
                if not self.buzzer_played:
                    self.play_buzzer()
                    self.buzzer_played = True
                self.set_rgb("red")
                self.stop_robot()

            elif self.detect_color == 'green':
                # 绿灯：开启巡线，绿色灯光，根据巡线逻辑动态控制机器人运动
                self.line_following_enabled = True
                self.set_rgb("green")
                self.buzzer_played = False
                # 巡线动作由巡线函数调用

            elif self.detect_color == 'yellow':
                # 黄灯：关闭巡线，黄色灯光，停止机器人
                self.set_rgb("yellow")
                self.line_following_enabled = False
                self.stop_robot()
                self.buzzer_played = False

            else:
                # 未识别颜色或其他色，关闭灯光，关闭蜂鸣器启用状态
                self.set_rgb("None")
                self.buzzer_played = False
                # 如果巡线使能，继续巡线，否则停止
                if self.line_following_enabled:
                    self.line_following()
                else:
                    self.stop_robot()

            # 当巡线使能时调用巡线控制
            if self.line_following_enabled:
                self.line_following()
            else:
                self.stop_robot()

        else:
            # 不运行游戏时全部停止，关闭灯光，巡线使能复原
            self.stop_robot()
            self.set_rgb("None")
            self.line_following_enabled = True
            self.buzzer_played = False

    def stop_robot(self):
        """停止机器人：停止所有运动指令"""
        stop_cmd = Twist()
        self.mecanum_pub.publish(stop_cmd)

    def move_forward(self):
        """机器人直行前进"""
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # 适中速度前进
        move_cmd.angular.z = 0.0
        self.mecanum_pub.publish(move_cmd)

    def line_following(self):
        """
        巡线逻辑（已将传感器信号逻辑反转）：
        现在True表示检测到黑线，False表示未检测到黑线，符合您的需求。
        """

        raw_data = self.line.readData()
        # 反转传感器状态，方便后续判断，True表示检测到黑线
        sensor_data = [not s for s in raw_data]

        self.get_logger().info(f"传感器状态（True检测到线）: S0={sensor_data[0]}, S1={sensor_data[1]}, S2={sensor_data[2]}, S3={sensor_data[3]}")

        # 根据反转后的信号状态控制运动
        if sensor_data[0] and not sensor_data[1] and sensor_data[2] and sensor_data[3]:
            self.get_logger().info("执行：右大转弯")
            self.sharp_right_turn()

        elif sensor_data[0] and  sensor_data[1] and  not sensor_data[2] and sensor_data[3]:
            self.get_logger().info("执行：左大转弯")
            self.sharp_left_turn()

        elif all(sensor_data):
            self.get_logger().info("所有传感器检测到黑线，停止机器人")
            self.stop_robot()

        elif sensor_data[0] and not sensor_data[1] and not sensor_data[2] and sensor_data[3]:
            self.get_logger().info("执行：直走")
            self.move_forward()

        elif not sensor_data[0] and not sensor_data[1]  and  sensor_data[2] and  sensor_data[3]:
            self.get_logger().info("执行：左转")
            self.turn_left()

        elif  sensor_data[0] and  sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
            self.get_logger().info("执行：右转")
            self.turn_right()

        elif sensor_data[0] and  sensor_data[1] and not sensor_data[2] and sensor_data[3]:
            self.get_logger().info("执行：右转")
            self.turn_right()

        elif sensor_data[0] and not sensor_data[1] and   sensor_data[2] and sensor_data[3]:
            self.get_logger().info("执行：左转")
            self.turn_left()

        elif  sensor_data[0] and not  sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
            self.get_logger().info("执行：右大转弯")
            self.sharp_right_turn()

        elif not sensor_data[0] and not sensor_data[1] and not sensor_data[2] and  sensor_data[3]:
            self.get_logger().info("执行：左大转弯")
            self.sharp_left_turn()

        elif  sensor_data[0] and not sensor_data[1] and not sensor_data[2] and sensor_data[3]:
            self.get_logger().info("执行：直走")
            self.move_forward()

        elif sensor_data[0] and sensor_data[1] and sensor_data[2] and sensor_data[3]:
            self.get_logger().info("所有传感器都检测到线，继续前进")
            self.move_forward()

        else:
            self.get_logger().info("传感器状态异常，停止机器人")
            self.stop_robot()

    def turn_right(self):
        """机器人向右小转弯，线偏右时调整"""
        move_cmd = Twist()
        move_cmd.linear.x = 0.3
        move_cmd.angular.z = -5.0
        self.mecanum_pub.publish(move_cmd)

    def turn_left(self):
        """机器人向左小转弯，线偏左时调整"""
        move_cmd = Twist()
        move_cmd.linear.x = 0.3
        move_cmd.angular.z = 5.0
        self.mecanum_pub.publish(move_cmd)

    def sharp_right_turn(self):
        """机器人右大转弯，会减速"""
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = -10.0
        self.mecanum_pub.publish(move_cmd)

    def sharp_left_turn(self):
        """机器人左大转弯，会减速"""
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 10.0
        self.mecanum_pub.publish(move_cmd)

    def set_rgb(self, color):
        """设置RGB灯颜色，根据色名字设定对应RGB值"""
        rgb_msg = RGBStates()
        rgb_state1 = RGBState()
        rgb_state1.index = 1
        rgb_state2 = RGBState()
        rgb_state2.index = 2

        if color == "red":
            rgb_state1.red, rgb_state1.green, rgb_state1.blue = 255, 0, 0
            rgb_state2.red, rgb_state2.green, rgb_state2.blue = 255, 0, 0
        elif color == "green":
            rgb_state1.red, rgb_state1.green, rgb_state1.blue = 0, 255, 0
            rgb_state2.red, rgb_state2.green, rgb_state2.blue = 0, 255, 0
        elif color == "yellow":
            rgb_state1.red, rgb_state1.green, rgb_state1.blue = 255, 255, 0
            rgb_state2.red, rgb_state2.green, rgb_state2.blue = 255, 255, 0
        else:  # 关闭灯光
            rgb_state1.red, rgb_state1.green, rgb_state1.blue = 0, 0, 0
            rgb_state2.red, rgb_state2.green, rgb_state2.blue = 0, 0, 0

        rgb_msg.states = [rgb_state1, rgb_state2]
        self.rgb_pub.publish(rgb_msg)
        self.get_logger().info(f"设置RGB灯为：{color}")

    def play_buzzer(self):
        """激活蜂鸣器"""
        buzzer_msg = BuzzerState()
        buzzer_msg.freq = 1000  # 频率1000Hz
        buzzer_msg.on_time = 0.1  # 响铃时间0.1秒
        buzzer_msg.off_time = 0.1  # 静音时间0.1秒
        buzzer_msg.repeat = 1  # 重复次数
        self.buzzer_pub.publish(buzzer_msg)
        self.get_logger().info("蜂鸣器响起")

    def image_callback(self, img_msg):
        """图像回调，转为OpenCV图像，处理颜色识别"""
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        frame = self.process_image(img)
        cv2.imshow('Processed Frame', frame)
        cv2.waitKey(1)

    def process_image(self, img):
        """颜色识别函数，更新detect_color状态"""
        img_copy = img.copy()
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        max_area = 0
        color_area_max = None
        max_contour = None

        for color in self.target_color:
            if color in self.lab_data:
                min_lab = tuple(self.lab_data[color]['min'])
                max_lab = tuple(self.lab_data[color]['max'])

                mask = cv2.inRange(frame_lab, min_lab, max_lab)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))

                contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                contour, area = self.get_area_max_contour(contours)

                if contour is not None and area > max_area:
                    max_area = area
                    color_area_max = color
                    max_contour = contour
        
        if max_area > 2500 and max_contour is not None:
            rect = cv2.minAreaRect(max_contour)
            box = np.intp(cv2.boxPoints(rect))
            cv2.drawContours(img, [box], -1, (0, 255, 0), 2)
            self.detect_color = color_area_max
        else:
            self.detect_color = "None"

        cv2.putText(img, "Color: " + self.detect_color, (10, img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)

        return img

    def get_area_max_contour(self, contours):
        """获取面积最大的轮廓及其面积"""
        area_max = 0
        contour_max = None
        for c in contours:
            area = abs(cv2.contourArea(c))
            if area > area_max and area > 300:  # 面积阈值避免噪声
                area_max = area
                contour_max = c
        return contour_max, area_max


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
