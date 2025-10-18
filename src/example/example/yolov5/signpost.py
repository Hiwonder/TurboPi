import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import sdk.FourInfrared as infrared
from std_msgs.msg import String
import json
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # 发布者设置
        self.mecanum_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.main_control_loop)
        self.bridge = CvBridge()
        self.size = (640, 480)
        self.line_following_enabled = True  # 启用寻线
        self.line = infrared.FourInfrared()

        # 订阅者设置 (detection_result topic)
        self.subscription = self.create_subscription(
            String,
            '/detection_result',
            self.detection_callback,
            10)
        self.subscription  # prevent unused variable warning

        # 状态变量
        self.current_command = "go"  # 默认前进
        self.command_counter = 0
        self.command_threshold = 3
        self.confidence_threshold = 0.75
        self.area_threshold = 91000.0
        self.action_duration = 0.2  # 执行时间 (秒)
        self.last_command = None


    def main_control_loop(self):
        # 先检查是否有指令，如果有，则执行指令，如果没有，则执行寻线
        if self.current_command != "go":
            if self.current_command == "right":
                # 执行右转命令，持续一段时间
                self.turn_right_and_line_follow()
                time.sleep(self.action_duration)
                self.stop_robot()

                # 重置状态，允许继续寻线
                self.current_command = "go"
                self.command_counter = 0
                self.last_command = None
                self.get_logger().info("Executed Right Turn Command for 0.2 seconds")  #0.2

            elif self.current_command == "park":
                # 执行停车命令，持续一段时间
                self.stop_robot_and_adjust()
                time.sleep(self.action_duration)
                self.stop_robot()

                # 重置状态，允许继续寻线
                self.current_command = "go"
                self.command_counter = 0
                self.last_command = None
                self.get_logger().info("Executed Stop Command (Park) for 0.2 seconds") #0.2

            elif self.current_command in ["green", "red"]:  # 处理 green 和 red
                # 执行前进命令，持续一段时间
                self.move_forward_and_line_follow()
                time.sleep(self.action_duration)
                self.stop_robot()

                # 重置状态，允许继续寻线
                self.current_command = "go"
                self.command_counter = 0
                self.last_command = None
                self.get_logger().info(f"Executed Move Forward Command ({self.current_command}) for 0.2 seconds")  # 输出指令类型

            else:
                # 未知指令，停止机器人并输出日志
                self.stop_robot()
                self.get_logger().warn(f"Unknown command received: {self.current_command}")

                # 重置状态，允许继续寻线
                self.current_command = "go"
                self.command_counter = 0
                self.last_command = None

        elif self.line_following_enabled: #寻线使能时，采用寻线
            self.line_following()
        else:
            self.stop_robot()  # 默认停止机器人

    def stop_robot(self):
        # 停止机器人
        stop_cmd = Twist()
        self.mecanum_pub.publish(stop_cmd)

    def move_forward(self):
        # 向前移动
        move_cmd = Twist()
        move_cmd.linear.x = 0.7 # 降低速度
        self.mecanum_pub.publish(move_cmd)

    def line_following(self):
        # 寻线功能
        sensor_data = self.line.readData()
        self.get_logger().info(f"Sensor 0: {sensor_data[0]}, Sensor 1: {sensor_data[1]},Sensor 2: {sensor_data[2]},Sensor 3: {sensor_data[3]}")

        # 情况 1: Sensor 0: True, Sensor 1: False, Sensor 2: True, Sensor 3: True
        if sensor_data[0] and not sensor_data[1] and sensor_data[2] and sensor_data[3]:
            self.sharp_right_turn()  # 执行右大转弯

        # 情况 2: Sensor 0: False, Sensor 1: True, Sensor 2: False, Sensor 3: False
        elif not sensor_data[0] and sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
            self.sharp_left_turn()  # 执行左大转弯

        # # 所有传感器都在线上，停止
        elif all(sensor_data):
            self.stop_robot()

        # 2, 3号传感器检测到黑线 (Sensors 2 and 3 detect the black line)
        elif sensor_data[0] and not sensor_data[1] and not sensor_data[2] and sensor_data[3]:
            self.move_forward()  # Move forward instead of backward

        # 0, 1号传感器检测到黑线 (Sensors 2 and 3 detect the black line)
        elif not sensor_data[0] and not sensor_data[1] and  sensor_data[2] and sensor_data[3]:
            self.turn_left()  # Move forward instead of backward

        elif sensor_data[0] and  sensor_data[1] and  not sensor_data[2] and not sensor_data[3]:
            self.turn_right()  # Move forward instead of backward

        # 3号传感器检测到黑线 (Sensor 3 detects the black line)
        elif sensor_data[0] and sensor_data[1] and not sensor_data[2] and sensor_data[3]:
            self.turn_right()  # Turn right

        # 2号传感器检测到黑线 (Sensor 2 detects the black line)
        elif sensor_data[0] and  not sensor_data[1] and  sensor_data[2] and sensor_data[3]:
            self.turn_left()  # Turn left

        # 4号传感器检测到黑线 (Sensor 4 detects the black line)
        elif sensor_data[0] and sensor_data[1] and sensor_data[2] and not sensor_data[3]:
            self.sharp_right_turn()  # Sharp right turn

        # 1号传感器检测到黑线 (Sensor 1 detects the black line)
        elif not sensor_data[0] and sensor_data[1] and sensor_data[2] and sensor_data[3]:
            self.sharp_left_turn()  # Sharp left turn

        # 添加新的条件
        # 传感器 0 和 1 都没有检测到黑线，传感器 2 和 3 检测到黑线 (Left turn)
        elif not sensor_data[0] and not sensor_data[1] and sensor_data[2] and sensor_data[3]:
            self.move_forward()  # 左转

        # 传感器 0 和 1 检测到黑线，传感器 2 和 3 检测到黑线 (Right turn)
        elif sensor_data[0] and sensor_data[1] and sensor_data[2] and sensor_data[3]:
            self.move_forward()  # 右转

        else:
            self.stop_robot()  # 默认停止机器人
            time.sleep(1.0)

    def turn_right(self):
        # 机器人右转
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # 降低速度
        move_cmd.angular.z = -8.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

    def turn_left(self):
        # 机器人左转
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # 降低速度
        move_cmd.angular.z = 8.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

    def sharp_right_turn(self):
        # 机器人右大转弯
        move_cmd = Twist()
        move_cmd.linear.x = 0.3  # 降低速度
        move_cmd.angular.z = -9.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

    def sharp_left_turn(self):
        # 机器人左大转弯
        move_cmd = Twist()
        move_cmd.linear.x = 0.3  # 降低速度
        move_cmd.angular.z = 9.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

    def detection_callback(self, msg):
        # 接收到消息后的处理逻辑
        detection_string = msg.data

        if not detection_string:
            self.get_logger().info('Received empty message, no object detected.')
            return

        self.get_logger().debug(f'Received raw detection string: {detection_string}')

        try:
            detections = json.loads(detection_string)

            if detections:
                detection = detections[0]
                label = detection['label']
                confidence = detection['confidence']
                area = detection['area']

                self.get_logger().info(f'Detected object: Label={label}, Confidence={confidence}, Area={area}')

                # 使用不同的面积阈值
                if label in ["green", "red"]:
                    area_threshold_for_command = 5000.0  #  green 和 red 使用的 area_threshold
                else:
                    area_threshold_for_command = self.area_threshold # 其他指令使用 self.area_threshold

                # 三次检测逻辑
                if confidence >= self.confidence_threshold and area >= area_threshold_for_command:
                    if self.last_command == label:
                        self.command_counter += 1
                    else:
                        self.last_command = label
                        self.command_counter = 1

                    if self.command_counter >= self.command_threshold:
                        if label in ["right"]:
                            self.current_command = "right"
                            self.get_logger().info("Right turn signal detected")
                        elif label in ["park", "red", "crosswalk"]:
                            self.current_command = "park"
                            self.get_logger().info("Stop signal detected (park, red, or crosswalk)")
                        elif label in ["go", "green"]:
                            self.current_command = "green"
                            self.get_logger().info("Go signal detected (go, green)")
                        else:
                            self.get_logger().warn(f"Unknown object detected: {label}")
            else:
                self.get_logger().info('Detection is empty, no object detected.')

        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON Decode Error: {e}")
        except KeyError as e:
            self.get_logger().warn(f"KeyError: Missing key {e}")



    def turn_right_and_line_follow(self):
        # 机器人右转, 不寻线
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # 降低速度，同时前进
        move_cmd.angular.z = -7.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

    def move_forward_and_line_follow(self): # 合并为 move_forward_and_line_follow
        # 机器人前进, 同时寻线
        start_time = time.time()
        while time.time() - start_time < self.action_duration:

            move_cmd = Twist()
            move_cmd.linear.x = 0.4 # 降低速度
            self.mecanum_pub.publish(move_cmd)

            sensor_data = self.line.readData()
            if sensor_data[0] and sensor_data[1] and not sensor_data[2] and sensor_data[3]:  # Slightly right
                move_cmd.angular.z = -1.0  # 微调角度
            elif sensor_data[0] and not sensor_data[1] and sensor_data[2] and sensor_data[3]:  # Slightly Left
                move_cmd.angular.z = 1.0
            self.mecanum_pub.publish(move_cmd) # Apply the adjustments
            rclpy.spin_once(self, timeout_sec=0.01) # Allow other callbacks to be processed

    def stop_robot_and_adjust(self):
       # 停止机器人并尝试调整回线上
        start_time = time.time()
        while time.time() - start_time < self.action_duration:  #尝试调整0.2秒
            stop_cmd = Twist()
            self.mecanum_pub.publish(stop_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)  #短暂等待，确保停止命令执行

            sensor_data = self.line.readData()

            # 如果不在线上，尝试调整
            if not any(sensor_data):
                self.get_logger().info("Not on line, attempting to adjust")

                adjust_cmd = Twist()  # Create a new Twist message for adjustments

                if sensor_data[0] and sensor_data[1] and not sensor_data[2] and sensor_data[3]:  # Slightly right
                    adjust_cmd.angular.z = -0.5  # Smaller adjustment
                    self.mecanum_pub.publish(adjust_cmd)
                    rclpy.spin_once(self, timeout_sec=0.1)
                    self.stop_robot()

                elif sensor_data[0] and not sensor_data[1] and sensor_data[2] and sensor_data[3]:  # Slightly Left
                    adjust_cmd.angular.z = 0.5  # Smaller Adjustment
                    self.mecanum_pub.publish(adjust_cmd)
                    rclpy.spin_once(self, timeout_sec=0.1)
                    self.stop_robot()
                else:
                    self.get_logger().info("Unable to adjust to line")
            else:
                break # 如果调整回到了线上，就退出循环

# ROS2 主函数
def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    # 保持节点循环执行
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
