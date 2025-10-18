#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import math
import numpy as np
import mediapipe as mp
from std_srvs.srv import SetBool, Trigger
from interfaces.srv import SetPoint, SetFloat64

from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
import threading  

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        self.bridge = CvBridge()

        # 订阅原始图像
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # 发布机器人运动指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 发布手势识别处理后的图像
        self.image_gesture_pub = self.create_publisher(Image, '/image_gesture_recognition', 10)

        # 初始化 Mediapipe 手势识别模型
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(model_complexity=0, 
                                          min_detection_confidence=0.5, 
                                          min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # 舵机控制发布器
        self.pwm_servo_pub = self.create_publisher(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', 10)
        
    

        # 状态变量
        self.gesture_num = None          # 当前识别的手势编号
        self.results_lock = False       
        self.results_list = []           
        self.gesture_control_enabled = False  
        self.gesture_game_active = False      

        # 服务1：进入游戏模式，仅发布图像，不启动手势识别
        self.game_enter_service = self.create_service(Trigger, 
                                                      '/gesture_game_control/enter', 
                                                      self.enter_game_mode)

        # 服务2：启动或停止手势识别，布尔型：true启动，false停止
        self.recognition_service = self.create_service(SetBool, 
                                                       '/gesture_control/set_running', 
                                                       self.set_gesture_recognition)

        self.get_logger().info("手势控制节点启动")

    def enter_game_mode(self, request, response):
        """
        进入游戏模式的服务回调
        调用该服务后，仅发布图像，手势识别功能不启动
        """
        self.gesture_game_active = True
        self.gesture_control_enabled = False  # 进入游戏模式时不启动手势识别
        self.results_lock = False
        self.results_list = []
        self.gesture_num = None

        response.success = True
        response.message = "已进入游戏模式，仅显示图像，手势识别未启动。"
        self.get_logger().info("进入游戏模式，仅显示图像")
        return response

    def set_gesture_recognition(self, request, response):
        """
        手势识别启动/停止服务回调
        请求：布尔型，true表示启动手势识别，false表示停止手势识别
        """
        if request.data:
            self.gesture_control_enabled = True
            response.success = True
            response.message = "手势识别已启动。"
            self.get_logger().info("手势识别已启动")
        else:
            self.gesture_control_enabled = False
            self.results_lock = False
            self.results_list = []
            self.gesture_num = None
            self.cmd_vel_pub.publish(Twist())  # 停止机器人运动
            response.success = True
            response.message = "手势识别已停止。"
            self.get_logger().info("手势识别已停止")
        return response

    def set_servo_position(self, servo_id, position, duration=0.0):
        """
        设置指定舵机位置，并设置运动持续时间
        """
        servo_state = PWMServoState()
        servo_state.id = [servo_id]
        servo_state.position = [position]
        set_pwm_servo_state_msg = SetPWMServoState()
        set_pwm_servo_state_msg.state = [servo_state]
        set_pwm_servo_state_msg.duration = duration  # 设置运动时间

        self.pwm_servo_pub.publish(set_pwm_servo_state_msg)
        self.get_logger().info(f"设置舵机 {servo_id} 到位置 {position}，持续 {duration} 秒")

    def vector_2d_angle(self, v1, v2):
        """
        计算二维向量 v1 与 v2 的夹角（单位：度）
        """
        v1_x, v1_y = v1
        v2_x, v2_y = v2
        try:
            angle_ = math.degrees(math.acos(
                (v1_x * v2_x + v1_y * v2_y) / (((v1_x ** 2 + v1_y ** 2) ** 0.5) * ((v2_x ** 2 + v2_y ** 2) ** 0.5))
            ))
        except Exception as e:
            angle_ = 65535.0
        if angle_ > 180.0:
            angle_ = 65535.0
        return angle_

    def hand_angle(self, hand_):
        """
        根据手部关键点坐标计算各手指的角度列表
        """
        angle_list = []
        # 拇指
        angle_ = self.vector_2d_angle(
            (int(hand_[0][0]) - int(hand_[2][0]), int(hand_[0][1]) - int(hand_[2][1])),
            (int(hand_[3][0]) - int(hand_[4][0]), int(hand_[3][1]) - int(hand_[4][1]))
        )
        angle_list.append(angle_)
        # 食指
        angle_ = self.vector_2d_angle(
            (int(hand_[0][0]) - int(hand_[6][0]), int(hand_[0][1]) - int(hand_[6][1])),
            (int(hand_[7][0]) - int(hand_[8][0]), int(hand_[7][1]) - int(hand_[8][1]))
        )
        angle_list.append(angle_)
        # 中指
        angle_ = self.vector_2d_angle(
            (int(hand_[0][0]) - int(hand_[10][0]), int(hand_[0][1]) - int(hand_[10][1])),
            (int(hand_[11][0]) - int(hand_[12][0]), int(hand_[11][1]) - int(hand_[12][1]))
        )
        angle_list.append(angle_)
        # 无名指
        angle_ = self.vector_2d_angle(
            (int(hand_[0][0]) - int(hand_[14][0]), int(hand_[0][1]) - int(hand_[14][1])),
            (int(hand_[15][0]) - int(hand_[16][0]), int(hand_[15][1]) - int(hand_[16][1]))
        )
        angle_list.append(angle_)
        # 小指
        angle_ = self.vector_2d_angle(
            (int(hand_[0][0]) - int(hand_[18][0]), int(hand_[0][1]) - int(hand_[18][1])),
            (int(hand_[19][0]) - int(hand_[20][0]), int(hand_[19][1]) - int(hand_[20][1]))
        )
        angle_list.append(angle_)
        return angle_list

    def gesture(self, angle_list):
        """
        根据手指角度判断手势类型，返回一个手势编号（1～6），若无法识别返回0
        """
        gesture_num = 0
        thr_angle = 65.0      # 手指伸直角度阈值
        thr_angle_s = 49.0    # 手指弯曲角度阈值
        thr_angle_thumb = 53.0  # 拇指特殊阈值
        if 65535.0 not in angle_list:
            if (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and \
               (angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
                gesture_num = 1
            elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and \
                 (angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
                gesture_num = 2
            elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and \
                 (angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
                gesture_num = 3
            elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and \
                 (angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
                gesture_num = 4
            elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and \
                 (angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
                gesture_num = 5
            elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and \
                 (angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
                gesture_num = 6
        return gesture_num

    def image_callback(self, msg):
        """
        图像回调函数
        1. 将ROS图像消息转换为OpenCV格式
        2. 如果手势识别启动，则调用 Mediapipe 处理图像，获取手势结果
           并将结果收集达到一定数量后，开启单独线程执行机器人控制命令（避免阻塞图像处理）
        3. 如果处于游戏模式，则无论是否进行识别，都发布处理后的图像
        """
        # 将 ROS 图像消息转换为 OpenCV 图像
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img_h, img_w = img.shape[:2]
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 如果启动了手势识别，则处理图像获得手势结果
        if self.gesture_control_enabled:
            results = self.hands.process(imgRGB)
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # 绘制手部骨架
                    self.mp_drawing.draw_landmarks(img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    # 将归一化坐标转换为图像坐标
                    hand_local = [(landmark.x * img_w, landmark.y * img_h) for landmark in hand_landmarks.landmark]
                    # 只有当没有正在执行控制命令时才收集识别结果
                    if hand_local and not self.results_lock:
                        angle_list = self.hand_angle(hand_local)
                        gesture_results = self.gesture(angle_list)
                        # 在图像上显示识别结果编号
                        cv2.putText(img, str(gesture_results), (20, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 100, 0), 3)
                        if gesture_results:
                            self.results_list.append(gesture_results)
                            # 收集到5个结果后计算平均值作为最终识别结果，并调用控制函数
                            if len(self.results_list) >= 5:
                                self.gesture_num = int(np.mean(np.array(self.results_list)))
                                self.results_lock = True
                                self.results_list = []
                                # 在单独线程中执行控制命令，避免阻塞图像回调
                                threading.Thread(target=self.control_robot, daemon=True).start()

        # 如果游戏模式激活，则发布处理后的图像（无论是否进行手势识别）
        if self.gesture_game_active:
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.image_gesture_pub.publish(image_msg)

    def control_robot(self):
        """
        根据识别到的手势编号控制机器人运动
        注意：此函数在单独线程中运行，不会阻塞图像处理回调
        """
        twist = Twist()
        if self.gesture_num == 1:
            twist.linear.x = 0.5
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5)
        elif self.gesture_num == 2:
            twist.linear.x = -0.5
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5)
        elif self.gesture_num == 3:
            twist.linear.y = 0.5
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5)
        elif self.gesture_num == 4:
            twist.linear.y = -0.5
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5)
        elif self.gesture_num == 5:
            twist.angular.z = 10.0
            self.cmd_vel_pub.publish(twist)
        elif self.gesture_num == 6:
            twist.angular.z = -10.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(3.0)

        # 发送停止命令
        self.cmd_vel_pub.publish(Twist())
        time.sleep(1)
        # 重置锁，允许下一次手势识别控制
        self.results_lock = False

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
