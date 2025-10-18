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
from app.common import Heart
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState  

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        self.bridge = CvBridge()

        # 订阅原始图像
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # 发布机器人运动指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 发布手势识别图像
        self.image_gesture_pub = self.create_publisher(Image, '/image_gesture_recognition', 10)

        # Mediapipe 手势识别模型
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # 舵机控制
        self.pwm_servo_pub = self.create_publisher(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', 10)
        
        # 使用 self.get_name() 获取节点名称
        Heart(self, self.get_name() + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包

        self.gesture_num = None
        self.results_lock = False
        self.results_list = []
        self.gesture_control_enabled = False  # 手势识别是否开启
        self.gesture_game_active = False  # 手势游戏是否激活

        # 添加服务，用于启用或禁用手势识别
        self.gesture_control_service = self.create_service(SetBool, '/gesture_control_enable', self.toggle_gesture_control)

        # 添加服务，用于进入和退出玩法
        self.game_control_service = self.create_service(SetBool, '/gesture_game_control', self.toggle_game_control)

        self.get_logger().info("Gesture Control Node Started")


    def toggle_gesture_control(self, request, response):
        """服务回调函数，启用或禁用手势识别"""
        if request.data:
            self.gesture_control_enabled = True
            response.success = True
            response.message = "Gesture control enabled"
        else:
            self.gesture_control_enabled = False
            response.success = True
            response.message = "Gesture control disabled."
        return response

    def toggle_game_control(self, request, response):
        """服务回调函数，用于进入和退出玩法"""
        if request.data:
            # 进入玩法
            self.gesture_game_active = True
            self.gesture_control_enabled = True

            # 启动时设置舵机位置
            self.set_servo_position(1, 1500,2.0)  # 1号舵机位置 1500
            self.set_servo_position(2, 1200,2.0)  # 2号舵机位置 1200

            response.success = True
            response.message = "Game mode entered. Gesture recognition enabled."
        else:
            # 退出玩法
            self.gesture_game_active = False
            self.gesture_control_enabled = False
            self.results_lock = False
            self.results_list = []
            self.gesture_num = None
            self.cmd_vel_pub.publish(Twist())  # 停止机器人运动

            # 退出时设置舵机位置
            self.set_servo_position(1, 1500,2.0)  # 1号舵机恢复到 1500
            self.set_servo_position(2, 1500,2.0)  # 2号舵机恢复到 1500

            response.success = True
            response.message = "Game mode exited. Gesture recognition disabled."
        return response

    def set_servo_position(self, servo_id, position, duration=0.0):
        """发布舵机控制消息，加入持续时间"""
        servo_state = PWMServoState()
        servo_state.id = [servo_id]
        servo_state.position = [position]
        set_pwm_servo_state_msg = SetPWMServoState()
        set_pwm_servo_state_msg.state = [servo_state]
        set_pwm_servo_state_msg.duration = duration  # 设置舵机运动时间

        self.pwm_servo_pub.publish(set_pwm_servo_state_msg)
        self.get_logger().info(f"Set servo {servo_id} position to {position} with duration {duration} seconds")

    def vector_2d_angle(self, v1, v2):
        v1_x = v1[0]
        v1_y = v1[1]
        v2_x = v2[0]
        v2_y = v2[1]
        try:
            angle_ = math.degrees(math.acos(
                (v1_x * v2_x + v1_y * v2_y) / (((v1_x ** 2 + v1_y ** 2) ** 0.5) * ((v2_x ** 2 + v2_y ** 2) ** 0.5)) ))
        except:
            angle_ = 65535.0
        if angle_ > 180.0:
            angle_ = 65535.0
        return angle_

    def hand_angle(self, hand_):
        angle_list = []
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0]) - int(hand_[2][0])), (int(hand_[0][1]) - int(hand_[2][1]))),
            ((int(hand_[3][0]) - int(hand_[4][0])), (int(hand_[3][1]) - int(hand_[4][1])) )
        )
        angle_list.append(angle_)
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0]) - int(hand_[6][0])), (int(hand_[0][1]) - int(hand_[6][1]))),
            ((int(hand_[7][0]) - int(hand_[8][0])), (int(hand_[7][1]) - int(hand_[8][1])) )
        )
        angle_list.append(angle_)
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0]) - int(hand_[10][0])), (int(hand_[0][1]) - int(hand_[10][1]))),
            ((int(hand_[11][0]) - int(hand_[12][0])), (int(hand_[11][1]) - int(hand_[12][1])) )
        )
        angle_list.append(angle_)
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0]) - int(hand_[14][0])), (int(hand_[0][1]) - int(hand_[14][1]))),
            ((int(hand_[15][0]) - int(hand_[16][0])), (int(hand_[15][1]) - int(hand_[16][1])) )
        )
        angle_list.append(angle_)
        angle_ = self.vector_2d_angle(
            ((int(hand_[0][0]) - int(hand_[18][0])), (int(hand_[0][1]) - int(hand_[18][1]))),
            ((int(hand_[19][0]) - int(hand_[20][0])), (int(hand_[19][1]) - int(hand_[20][1])) )
        )
        angle_list.append(angle_)
        return angle_list

    def gesture(self, angle_list):
        gesture_num = 0
        thr_angle = 65.0
        thr_angle_s = 49.0
        thr_angle_thumb = 53.0
        if 65535.0 not in angle_list:
            if (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
                    angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
                gesture_num = 1
            elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                    angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
                gesture_num = 2
            elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                    angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
                gesture_num = 3
            elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                    angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
                gesture_num = 4
            elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                    angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
                gesture_num = 5
            elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
                    angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
                gesture_num = 6
        return gesture_num

    def image_callback(self, msg):
        # 只有开启手势识别时才会进行识别，否则只显示画面
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img_h, img_w = img.shape[:2]
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 如果开启了手势识别，才进行手势处理
        if self.gesture_control_enabled:
            results = self.hands.process(imgRGB)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    hand_local = [(landmark.x * img_w, landmark.y * img_h) for landmark in hand_landmarks.landmark]
                    if hand_local:
                        angle_list = self.hand_angle(hand_local)
                        gesture_results = self.gesture(angle_list)
                        cv2.putText(img, str(gesture_results), (20, 50), 0, 2, (255, 100, 0), 3)
                        if gesture_results:
                            self.results_list.append(gesture_results)
                            if len(self.results_list) == 5:
                                self.gesture_num = int(np.mean(np.array(self.results_list)))
                                self.results_lock = True
                                self.results_list = []
                                self.control_robot()

        # 只有在游戏模式激活时，才发布图像
        if self.gesture_game_active:
            # 发布手势识别后的图像（无论识别与否，都会发布图像）
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.image_gesture_pub.publish(image_msg)

    def control_robot(self):
        twist = Twist()
        if self.gesture_num == 1:
            twist.linear.x = 0.5
            time.sleep(0.5)
        elif self.gesture_num == 2:
            twist.linear.x = -0.5
            time.sleep(0.5)
        elif self.gesture_num == 3:
            twist.linear.y = 0.5
            time.sleep(0.5)
        elif self.gesture_num == 4:
            twist.linear.y = -0.5
            time.sleep(0.5)
        elif self.gesture_num == 5:
            twist.angular.z = 10.0
            
        elif self.gesture_num == 6:
            twist.angular.z = -10.0
            time.sleep(3.0)

        self.cmd_vel_pub.publish(twist)
        time.sleep(1)
        self.cmd_vel_pub.publish(Twist())  # 停止机器人
        self.results_lock = False

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
