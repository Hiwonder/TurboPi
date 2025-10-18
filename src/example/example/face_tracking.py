#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import rclpy
import queue
import threading
import numpy as np
import mediapipe as mp
import sdk.pid as pid
import sdk.yaml_handle as yaml_handle
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from ros_robot_controller_msgs.msg import MotorsState, SetPWMServoState, PWMServoState

class FaceMeshNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.bridge = CvBridge()
        
        # pid初始化(pid initialization)
       	self.car_x_pid = pid.PID(P=0.150, I=0.001, D=0.0001)
        self.car_y_pid = pid.PID(P=0.002, I=0.001, D=0.0001)
        self.servo_x_pid = pid.PID(P=0.1, I=0.0000, D=0.0000) 
        self.servo_y_pid = pid.PID(P=0.1, I=0.000, D=0.000)
        
        self.car_en = False
        self.is_running = False
        self.img_h, self.img_w = 0, 0
        self.center_x, self.center_y, self.area = -1, -1, 0
        self.lock = threading.RLock()
        
        
        self.servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
        
        self.servo1 = self.servo_data['servo1'] - 350
        self.servo2 = self.servo_data['servo2']
        self.servo_x = self.servo2
        self.servo_y = self.servo1       


        # 导入人脸识别模块(import face recognition module)
        self.face = mp.solutions.face_detection
        # 自定义人脸识别方法，最小的人脸检测置信度0.5(customize face recognition method, with minimum face detection confidence of 0.5)
        self.faceDetection = self.face.FaceDetection(min_detection_confidence=0.5)
        
        
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the image processing result)
        self.mecanum_pub = self.create_publisher(Twist, 'cmd_vel', 1)  # 底盘控制(chassis control)
        self.pwm_pub = self.create_publisher(SetPWMServoState,'ros_robot_controller/pwm_servo/set_state',10)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)        

        self.debug = True
        self.image_queue = queue.Queue(maxsize=2)
        self.pwm_controller([1, self.servo_y], [2, self.servo_x])
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()

    def reset_value(self):
        '''
        重置参数(reset parameter)
        :return:
        '''
        self.center_x, self.center_y, self.area = -1, -1, 0
        self.__isRunning = False
        
        self.car_x_pid.clear()
        self.car_y_pid.clear()
        self.servo_x_pid.clear()
        self.servo_y_pid.clear()
        
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
        
    
    
    def image_callback(self, ros_image):
    
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        
        img_copy = rgb_image.copy()
        
        self.img_h, self.img_w = rgb_image.shape[:2]
        
        imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB) # 将BGR图像转为RGB图像(convert the BGR image to RGB image)
        results = self.faceDetection.process(imgRGB) # 将每一帧图像传给人脸识别模块(pass each frame image to the face recognition module)
        if results.detections:   # 如果检测不到人脸那就返回None(return None if face is not detected)
            for index, detection in enumerate(results.detections): # 返回人脸索引index(第几张脸)，和关键点的坐标信息(return face index (which face) and coordinates of key points)
                bboxC = detection.location_data.relative_bounding_box # 设置一个边界框，接收所有的框的xywh及关键点信息(set up a bounding box to receive xywh (X-axis, Y-axis, width, height) and key information for all boxes)
                
                # 将边界框的坐标点,宽,高从比例坐标转换成像素坐标(convert the coordinates, width, and height of the bounding box from relative coordinates to pixel coordinates)
                bbox = (int(bboxC.xmin * self.img_w), int(bboxC.ymin * self.img_h),  
                       int(bboxC.width * self.img_w), int(bboxC.height * self.img_h))
                cv2.rectangle(rgb_image, bbox, (0,255,0), 2)  # 在每一帧图像上绘制矩形框(draw rectangular boxes on each frame image)
                x, y, w, h = bbox  # 获取识别框的信息,xy为左上角坐标点(get information of the recognition box, where xy represents the coordinates of the upper-left corner)
                self.center_x =  int(x + (w/2))
                self.center_y =  int(y + (h/2))
                self.area = int(w * h)
        else:
            self.center_x, self.center_y, self.area = -1, -1, 0
            
            
        if self.debug:
            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
                self.image_queue.get()
                # 将图像放入队列(put the image into the queue)
            self.image_queue.put(rgb_image)
        else:           
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(rgb_image, "rgb8"))
            
    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.is_running = request.data
            if not self.is_running:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response

        
        

    def main(self):
        while True:
            twist = Twist()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)

            cv2.imshow('face_landmarker', image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                break
            
            if self.is_running:                
                if self.center_x != -1 and self.center_y != -1:  
                                  
                    # 摄像头云台追踪(camera pan-tilt tracking)
                    # 根据摄像头X轴坐标追踪(track based on the camera X-axis coordinates)
                    if abs(self.center_x - self.img_w/2.0) < 15: # 移动幅度比较小，则不需要动(if the movement amplitude is small, no action is required)
                        self.center_x = self.img_w/2.0
                    self.servo_x_pid.SetPoint = self.img_w/2.0 # 设定(set)
                    self.servo_x_pid.update(self.center_x)     # 当前(current)
                    self.servo_x += int(self.servo_x_pid.output)  # 获取PID输出值(get PID output value)
                    
                    self.servo_x = 800 if self.servo_x < 800 else self.servo_x # 设置舵机范围(set servo range)
                    self.servo_x = 2200 if self.servo_x > 2200 else self.servo_x
                    
                    # 根据摄像头Y轴坐标追踪(track based on the camera Y-axis coordinates)
                    if abs(self.center_y - self.img_h/2.0) < 10: # 移动幅度比较小，则不需要动(if the movement amplitude is small, no action is required)
                        self.center_y = self.img_h/2.0
                    self.servo_y_pid.SetPoint = self.img_h/2.0  
                    self.servo_y_pid.update(self.center_y)
                    self.servo_y -= int(self.servo_y_pid.output) # 获取PID输出值(gei PID output value)
                    
                    self.servo_y = 1000 if self.servo_y < 1000 else self.servo_y # 设置舵机范围(set servo range)
                    self.servo_y = 1900 if self.servo_y > 1900 else self.servo_y
                    self.pwm_controller([1, self.servo_y], [2, self.servo_x]) # 设置舵机移动(set servo movement)
                    
                    
                    # 车身跟随追踪(vehicle following tracking)
                    # 根据目标大小进行远近追踪(distance tracking based on the target size)
                    if abs(self.area - 30000) < 2000 or self.servo_y < 1100:
                        self.car_y_pid.SetPoint = self.area
                    else:
                        self.car_y_pid.SetPoint = 30000
                    self.car_y_pid.update(self.area)
                    dy = self.car_y_pid.output   # 获取PID输出值(get PID output value)
                    dy = 0 if abs(dy) < 20 else dy # 设置速度范围(set velocity range)
                    
                    # 根据X轴舵机值进行追踪(track based on X-axis servo value)
                    if abs(self.servo_x - self.servo2) < 15:
                        self.car_x_pid.SetPoint = self.servo_x
                    else:
                        self.car_x_pid.SetPoint = self.servo2
                    self.car_x_pid.update(self.servo_x)
                    dx = self.car_x_pid.output   # 获取PID输出值(get PID output value)
                    dx = 0 if abs(dx) < 20 else dx # 设置速度范围(set velocity range)
                   
                    twist.linear.x = dx / 100
                    twist.linear.y = dy / 100
                    
                    self.mecanum_pub.publish(twist)
                    
                    self.car_en = True
                else:
                    if self.car_en:                       
                        self.car_en = False
                        self.mecanum_pub.publish(Twist())  
            else:
                if self.car_en: 
                    self.car_en = False
                    self.mecanum_pub.publish(Twist())                                       
                
              
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    node = FaceMeshNode('face_landmarker')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == "__main__":
    main()
