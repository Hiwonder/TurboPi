#!/usr/bin/env python3
# encoding: utf-8
# 巡线(line following)
import os
import cv2
import math
import time
import rclpy
import queue
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
import sdk.yaml_handle as yaml_handle
from rclpy.node import Node
from app.common import Heart,ColorPicker
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import Image
from interfaces.srv import SetPoint, SetFloat64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from large_models_msgs.srv import SetString
from ros_robot_controller_msgs.msg import MotorsState, SetPWMServoState, PWMServoState,RGBState,RGBStates 

class LineFollower:
    def __init__(self, color, node, set_color, set_status = False):
        self.node = node
        self.set_status = set_status
        self.set_color = set_color
        self.lab_data = None
        if color is not None:
            self.target_lab, self.target_rgb = color
            
        self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path )
        self.rois = ((0.9, 0.96, 0, 1, 0.6), (0.71, 0.79, 0, 1, 0.3), (0.5, 0.58, 0, 1, 0.1))
        self.weight_sum = 1.0

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓(get the contour of the largest area)
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def __call__(self, image, result_image, threshold):
        centroid_sum = 0
        h, w = image.shape[:2]
        
        if self.set_status == False:              
            min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                         int(self.target_lab[1] - 50 * threshold),
                         int(self.target_lab[2] - 50 * threshold)]
            max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                         int(self.target_lab[1] + 50 * threshold),
                         int(self.target_lab[2] + 50 * threshold)]
            target_color = self.target_lab, min_color, max_color
        else:
            min_color = [self.lab_data[self.set_color]['min'][0],
                         self.lab_data[self.set_color]['min'][1],
                         self.lab_data[self.set_color]['min'][2]]
            max_color = [self.lab_data[self.set_color]['max'][0],
                         self.lab_data[self.set_color]['max'][1],
                         self.lab_data[self.set_color]['max'][2]]                         
            target_color = 0, min_color, max_color
            
        for roi in self.rois:
            blob = image[int(roi[0]*h):int(roi[1]*h), int(roi[2]*w):int(roi[3]*w)]  # 截取roi(intercept roi)
            img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb into lab)
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(perform Gaussian filtering to reduce noise)
            mask = cv2.inRange(img_blur, tuple(target_color[1]), tuple(target_color[2]))  # 二值化(image binarization)
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrode)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
            # cv2.imshow('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find the contour)
            max_contour_area = self.get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓(get the contour corresponding to the largest contour)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(minimum circumscribed rectangle)
                box = np.intp(cv2.boxPoints(rect))  # 四个角(four corners)
                for j in range(4):
                    box[j, 1] = box[j, 1] + int(roi[0]*h)
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of four points)

                # 获取矩形对角点(acquire the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点(center point of the line)
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)   # 画出中心点(draw the center point)
                centroid_sum += line_center_x * roi[-1]
        if centroid_sum == 0:
            return result_image, None
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点(calculate the center point according to the ratio)
        deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))   # 计算线角度(calculate the line angle)
        return result_image, deflection_angle

class LineFollowingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.name = name
        self.set_callback = False
        self.is_running = False
        self.color_picker = None
        self.follower = None
        self.scan_angle = math.radians(45)
        self.pid = pid.PID(0.005, 0.001, 0.0)
        self.stop = False
        self.threshold = 0.5
        self.stop_threshold = 200 # (mm)
        self.lock = threading.RLock()
        self.set_model = False
        self.exit_funcation = False
        self.__target_color = None
        self.image_sub = None
        self.sonar_sub = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.last_target_time = None
        self.heart = None  #  心跳包对象

        
        self.image_queue = queue.Queue(2)
        self.sonar_queue = queue.Queue(5)
        self.servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
        self.debug = self.get_parameter('debug').value
        self.sonar_rgb_pub = self.create_publisher(RGBStates, 'sonar_controller/set_rgb', 10)
        self.pwm_pub = self.create_publisher(SetPWMServoState,'ros_robot_controller/pwm_servo/set_state',10)
        self.mecanum_pub = self.create_publisher(Twist, 'cmd_vel', 1)  # 底盘控制(chassis control)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the image processing result)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 退出玩法(exit the game)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)
        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)  # 设置颜色(set the color)
        
        self.create_service(SetString, '~/set_large_model_target_color', self.set_large_model_target_color_srv_callback)  # 设置大模型玩法颜色(set the color)
        
        self.create_service(Trigger, '~/get_target_color', self.get_target_color_srv_callback)   # 获取颜色(get the color)
        self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_srv_callback)  # 设置阈值(set the threshold)
        self.heart = Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        
        #self.debug = False
        
        if self.debug: 
            threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

        # 添加超时计数器
        self.no_target_time = 0
        self.no_target_threshold = 5 # seconds

        self.exit_allowed = True  # 添加标志位，允许执行 exit_srv_callback
        
    def pwm_controller(self, *position_data):
        pwm_list = []
        msg = SetPWMServoState()
        msg.duration = 0.2
        for position in position_data:
            pos = PWMServoState()
            pos.id = [position[0]]
            pos.position = [int(position[1])]
            pwm_list.append(pos)
        msg.state = pwm_list
        self.pwm_pub.publish(msg)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def main(self):
        while True:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow("result", result)
            if self.debug and not self.set_callback:
                self.set_callback = True
                # 设置鼠标点击事件的回调函数(set a callback function for mouse click event)
                cv2.setMouseCallback("result", self.mouse_callback)
            k = cv2.waitKey(1)
            if k != -1:
                break
        self.mecanum_pub.publish(Twist())
        rclpy.shutdown()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info("x:{} y{}".format(x, y))
            msg = SetPoint.Request()
            if self.image_height is not None and self.image_width is not None:
                msg.data.x = x / self.image_width
                msg.data.y = y / self.image_height
                self.set_target_color_srv_callback(msg, SetPoint.Response())

    def sonar_rgb_controller(self):
        led1 = RGBState()
        led1.index = 0
        led1.red = 0
        led1.green = 0
        led1.blue = 0

        led2 = RGBState()
        led2.index = 1
        led2.red = 0
        led2.green = 0
        led2.blue = 0
        msg = RGBStates()
        msg.states = [led1,led2]
        self.sonar_rgb_pub.publish(msg)
    
    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "line following enter")
        self.pwm_controller([1, 1850], [2, 1500])
        self.sonar_rgb_controller()
        with self.lock:
            self.stop = False            
            self.is_running = False
            self.color_picker = None
            self.pid = pid.PID(3.1, 0.0, 0.0)
            self.follower = None
            self.threshold = 0.5
            if self.image_sub is None:
                 self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
            if self.sonar_sub is None:
                qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
                self.sonar_sub = self.create_subscription(Int32, 'sonar_controller/get_distance', self.sonar_callback, qos)  # subscribe to sonar data
            self.mecanum_pub.publish(Twist())
            # 重置超时计数器
            self.no_target_time = 0
            self.exit_allowed = True  # 允许执行 exit_srv_callback
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "line following exit")

        with self.lock:
            if not self.exit_allowed:  # 检查是否允许执行
                self.get_logger().info("exit_srv_callback blocked by flag.")
                response.success = True
                response.message = "exit blocked"
                return response

            self.exit_allowed = False  # 阻止再次执行

            # 以下是原始的退出服务逻辑
            self.pwm_controller([1, 1500], [2, 1500])
            try:
                if self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                    self.image_sub = None
                if self.sonar_sub is not None:
                    self.destroy_subscription(self.sonar_sub)
                    self.sonar_sub = None
            except Exception as e:
                self.get_logger().error(str(e))

            self.is_running = False
            self.color_picker = None
            self.pid = pid.PID(0.00, 0.001, 0.0)
            self.follower = None
            self.threshold = 0.5
            self.mecanum_pub.publish(Twist())
            self.no_target_time = 0
            # 停止并清理心跳包
            if self.heart is not None:
                self.heart.destroy()
                self.heart = None

        response.success = True
        response.message = "exit"
        return response

    # 取色巡线调用接口
    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_target_color")
        with self.lock:
            self.set_model = False
            self.__target_color = None
            x, y = request.data.x, request.data.y
            self.follower = None
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                self.color_picker = ColorPicker(request.data, 5)
                self.mecanum_pub.publish(Twist())
                self.no_target_time = 0
        response.success = True
        response.message = "set_target_color"
        return response
    
    # 大模型巡线调用接口    
    def set_large_model_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_large_model_target_color")
        with self.lock:           
            self.__target_color = request.data
            self.set_model = True
            self.mecanum_pub.publish(Twist())
            self.no_target_time = 0
            self.exit_funcation = True
        response.success = True
        return response

    def get_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "get_target_color")
        response.success = False
        response.message = "get_target_color"
        with self.lock:
            if self.follower is not None:
                response.success = True
                rgb = self.follower.target_rgb
                response.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return response

    # 开启玩法
    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.is_running = request.data
            if not self.is_running:
                self.mecanum_pub.publish(Twist())
            if self.is_running:
                self.no_target_time = 0
        response.success = True
        response.message = "set_running"
        return response

    def set_threshold_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set threshold")
        with self.lock:
            self.threshold = request.data
            response.success = True
            response.message = "set_threshold"
            return response

    # 超声波回调函数
    def sonar_callback(self, msg):           
        sonar_distance = msg.data        
        if self.sonar_queue.full():
            self.sonar_queue.get()      
        self.sonar_queue.put(sonar_distance)
        queue_list = list(self.sonar_queue.queue)
        sonar_distance_filtered = sum(queue_list) / len(queue_list)
        
        if sonar_distance_filtered <= self.stop_threshold:
            self.stop = True
        else: 
            self.stop = False        

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]
        result_image = np.copy(rgb_image)  # 显示结果用的画面 (the image used to display the result)
        target_detected = False # 标志是否检测到目标

        with self.lock:
            # 颜色拾取器和识别巡线互斥, 如果拾取器存在就开始拾取(color picker and line recognition are exclusive. If there is color picker, start picking)
            if self.color_picker is not None:  # 拾取器存在(color picker exists)
                try:
                    target_color, result_image = self.color_picker(rgb_image, result_image)
                    if target_color is not None:
                        self.color_picker = None
                        self.follower = LineFollower(target_color, self, None, False) 
                        self.get_logger().info("target color: {}".format(target_color))
                        target_detected = True # 成功拾取颜色，认为检测到目标
                except Exception as e:
                    self.get_logger().error(str(e))
                    
            # 大模型巡线功能
            elif self.color_picker is None and self.set_model:  
                self.set_model = False  
                self.follower = LineFollower(None, self, self.__target_color, True) 
                self.get_logger().info("self.follower:{}".format(self.follower))
                target_detected = True                         
            else:
                twist = Twist()
                twist.linear.x = 0.3
                if self.follower is not None:                   
                    try:
                        result_image, deflection_angle = self.follower(rgb_image, result_image, self.threshold)
                        if deflection_angle is not None and self.is_running and not self.stop:                            
                            self.pid.update(deflection_angle)                           
                            twist.angular.z = -common.set_range(self.pid.output, -8.0, 8.0)
                            self.mecanum_pub.publish(twist)
                            target_detected = True # 检测到巡线角度，认为检测到目标
                            self.last_target_time = time.time()

                        elif self.stop:
                            self.mecanum_pub.publish(Twist())
                        else:
                            self.pid.clear()
                    except Exception as e:
                        self.get_logger().error(str(e))

            # 超时检查
            if target_detected:
                self.no_target_time = 0  # 重置计数器
            else:               
                if self.is_running:
                    if time.time() - self.last_target_time > self.no_target_threshold and self.exit_funcation:  
                        self.exit_funcation = False
                        self.get_logger().warn("未检测到目标超过5秒，退出巡线")
                        self.exit_srv_callback(Trigger.Request(), Trigger.Response())  # 退出
                        return  # 提前返回，避免后续不必要的操作

        if self.debug:
            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
                self.image_queue.get()
                # 将图像放入队列(put the image into the queue)
            self.image_queue.put(result_image)
        else:
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

    def destroy_node(self):
        if self.heart is not None:
            self.heart.destroy()
            self.heart = None
        super().destroy_node()

def main():
    node = LineFollowingNode('line_following')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
