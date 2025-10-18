#!/usr/bin/env python3
# encoding: utf-8
# 二维码识别

import os
import cv2
import queue
import rclpy
import threading
import numpy as np
import time
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from app.common import Heart
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState


class QRCodeDetectNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('debug', False)
        self.name = name
        self.is_running = False
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.image_queue = queue.Queue(2)
        self.exit_allowed = True  # 添加标志位，允许执行 exit_srv_callback
        self.heart = None # 心跳包对象
        # self._timers = []  # 移除定时器列表
        threading.Thread(target=self.main, daemon=True).start()

        self.debug = self.get_parameter('debug').value

        callback_group = ReentrantCallbackGroup()

        self.mecanum_pub = self.create_publisher(Twist, '/cmd_vel', 1,
                                                 callback_group=callback_group)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1,
                                                      callback_group=callback_group)
        self.enter_srv = self.create_service(Trigger, '~/enter',
                                             self.enter_srv_callback,
                                             callback_group=callback_group)
        self.exit_srv = self.create_service(Trigger, '~/exit',
                                            self.exit_srv_callback,
                                            callback_group=callback_group)
        self.start_recognition_srv = self.create_service(SetBool,
                                                         '~/start_recognition',
                                                         self.start_recognition_srv_callback,
                                                         callback_group=callback_group)
        self.set_running_srv = self.create_service(SetBool, '~/set_running',
                                                    self.set_running_srv_callback,
                                                    callback_group=callback_group)
        self.pwm_servo_pub = self.create_publisher(SetPWMServoState,
                                                    '/ros_robot_controller/pwm_servo/set_state',
                                                    10)

        # 启动心跳包功能
        self.heart = Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(
            request=Trigger.Request(), response=Trigger.Response()))

        self.create_service(Trigger, '~/init_finish', self.get_node_state,
                            callback_group=callback_group)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def main(self):
        while self.is_running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
                result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                # cv2.imshow("result", result)
                # cv2.waitKey(1)  # 使得图像实时刷新
            except queue.Empty:
                if not self.is_running:
                    break
                else:
                    continue

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'QRCode detect enter')
        self.set_servo_position(1, 1200, 2.0)
        self.set_servo_position(2, 1500, 2.0)
        with self.lock:
            self.is_running = True
            self.exit_allowed = True # 允许执行 exit_srv_callback
            if self.image_sub is None:
                self.image_sub = self.create_subscription(Image, '/image_raw',
                                                        self.image_callback, 1)  # 摄像头订阅
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'QRCode exit')

        with self.lock:
            if not self.exit_allowed:  # 检查是否允许执行
                self.get_logger().info("exit_srv_callback blocked by flag.")
                response.success = True
                response.message = "exit blocked"
                return response

            self.exit_allowed = False  # 阻止再次执行

            # 1. 设置 is_running 标志位
            self.is_running = False

            # 2. 停止发布运动指令
            self.mecanum_pub.publish(Twist())

            # 3. 清空图像队列
            with self.image_queue.mutex:
                self.image_queue.queue.clear()

            # 4. 停止订阅图像话题
            try:
                if self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                    self.image_sub = None
            except Exception as e:
                self.get_logger().error(f"Error destroying subscription: {e}")

            # 5. 舵机回到初始位置
            self.set_servo_position(1, 1500, 2.0)
            self.set_servo_position(2, 1500, 2.0)

            # 6. 销毁心跳包
            if self.heart is not None:
                self.heart.destroy()
                self.heart = None

            response.success = True
            response.message = "exit"
            return response

    def start_recognition_srv_callback(self, request, response):
        """服务：开启/停止二维码识别"""
        with self.lock:
            self.is_running = request.data  # 启动或停止二维码识别
            if self.is_running:
                self.get_logger().info("二维码识别已启动")
            else:
                self.get_logger().info("二维码识别已停止")
                self.mecanum_pub.publish(Twist())  # 停止控制机器人运动

        response.success = True
        response.message = "Start recognition" if self.is_running else "Stop recognition"
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set running')
        with self.lock:
            self.is_running = request.data  # 启用或停止识别

            if self.is_running:
                self.get_logger().info("二维码识别已启动")
            else:
                self.get_logger().info("二维码识别已停止")
                self.mecanum_pub.publish(Twist())  # 停止控制机器人运动

        response.success = True
        response.message = "set running"
        return response

    def set_servo_position(self, servo_id, position, duration=0.0):
        """发布舵机控制消息，加入持续时间"""
        servo_state = PWMServoState()
        servo_state.id = [servo_id]
        servo_state.position = [position]
        set_pwm_servo_state_msg = SetPWMServoState()
        set_pwm_servo_state_msg.state = [servo_state]
        set_pwm_servo_state_msg.duration = duration

        self.pwm_servo_pub.publish(set_pwm_servo_state_msg)
        self.get_logger().info(
            f"Set servo {servo_id} position to {position} with duration {duration} seconds")

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]

        result_image = np.copy(rgb_image)  # 显示结果用的画面
        with self.lock:
            if self.is_running:
                decoded_objects = pyzbar.decode(result_image)
                for obj in decoded_objects:
                    points = obj.polygon
                    if len(points) == 4:
                        pts = np.array(points, dtype=np.int32)
                        cv2.polylines(result_image, [pts], isClosed=True, color=(0, 255, 0),
                                        thickness=2)
                    else:
                        hull = cv2.convexHull(np.array([point for point in obj.polygon],
                                                        dtype=np.float32))
                        cv2.polylines(result_image, [np.int32(hull)], isClosed=True,
                                        color=(0, 255, 0), thickness=3)
                    cv2.putText(result_image, obj.data.decode("utf-8"),
                                (obj.rect[0], obj.rect[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    qr_data = obj.data.decode("utf-8")
                    self.control_robot_based_on_qr(qr_data)

        if self.debug:
            if self.image_queue.full():
                self.image_queue.get()  # 丢弃最旧的图像
            self.image_queue.put(result_image)
        else:
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

    def control_robot_based_on_qr(self, qr_data):
        if qr_data == "1":
            # 前进2秒，使用 time.sleep
            self.move_forward(2)
        elif qr_data == "2":
            self.move_backward(2)
        elif qr_data == "3":
            self.move_right(2)
        elif qr_data == "4":
            self.move_left(2)

    def move_forward(self, duration):
        twist = Twist()
        twist.linear.x = 0.6  # 前进速度
        self.mecanum_pub.publish(twist)
        self.get_logger().info(f"Moving forward for {duration} seconds")
        time.sleep(duration)
        self.stop_robot()

    def move_backward(self, duration):
        twist = Twist()
        twist.linear.x = -0.6  # 后退速度
        self.mecanum_pub.publish(twist)
        self.get_logger().info(f"Moving backward for {duration} seconds")
        time.sleep(duration)
        self.stop_robot()

    def move_right(self, duration):
        twist = Twist()
        twist.linear.y = -0.6  # 右平移速度
        self.mecanum_pub.publish(twist)
        self.get_logger().info(f"Moving right for {duration} seconds")
        time.sleep(duration)
        self.stop_robot()

    def move_left(self, duration):
        twist = Twist()
        twist.linear.y = 0.6  # 左平移速度
        self.mecanum_pub.publish(twist)
        self.get_logger().info(f"Moving left for {duration} seconds")
        time.sleep(duration)
        self.stop_robot()

    def stop_robot(self):
        """停止机器人的运动"""
        twist = Twist()
        self.mecanum_pub.publish(twist)
        self.get_logger().info("Robot stopped")


def main():
    rclpy.init()  # 初始化
    node = QRCodeDetectNode('qrcode')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
