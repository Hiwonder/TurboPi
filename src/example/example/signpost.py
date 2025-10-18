import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import sdk.FourInfrared as infrared

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # 发布者设置
        self.mecanum_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.main_control_loop)
        self.bridge = CvBridge()
        self.size = (640, 480)
        self.line_following_enabled = True
        self.line = infrared.FourInfrared()
    def main_control_loop(self):
        if self.line_following_enabled:
            self.line_following()
        else:
            self.stop_robot()

    def stop_robot(self):
        # 停止机器人
        stop_cmd = Twist()
        self.mecanum_pub.publish(stop_cmd)

    def move_forward(self):
        # 向前移动
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # 降低速度
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

        # # 都在线外，也停止，避免原地转圈 (None of the sensors detect line, stop to avoid spinning in place)
        # elif not any(sensor_data):
        #     self.stop_robot()

        # # 2, 3号传感器检测到黑线 (Sensors 2 and 3 detect the black line)
        # elif not sensor_data[0] and sensor_data[1] and sensor_data[2] and not sensor_data[3]:
        #     self.move_forward()  # Move forward

        # # 3号传感器检测到黑线 (Sensor 3 detects the black line)
        # elif not sensor_data[0] and not sensor_data[1] and sensor_data[2] and not sensor_data[3]:
        #     self.turn_right()  # Turn slightly right

        # # 2号传感器检测到黑线 (Sensor 2 detects the black line)
        # elif not sensor_data[0] and sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
        #     self.turn_left()  # Turn slightly left

        # # 4号传感器检测到黑线 (Sensor 4 detects the black line)
        # elif not sensor_data[0] and not sensor_data[1] and not sensor_data[2] and sensor_data[3]:
        #     self.sharp_right_turn()  # Sharp right turn

        # # 1号传感器检测到黑线 (Sensor 1 detects the black line)
        # elif sensor_data[0] and not sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
        #     self.sharp_left_turn()  # Sharp left turn
        # else:
    
        # # else:
        #     self.stop_robot()

    def turn_right(self):
        # 机器人右转
        move_cmd = Twist()
        move_cmd.linear.x = 0.3  # 降低速度
        move_cmd.angular.z = -5.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

    def turn_left(self):
        # 机器人左转
        move_cmd = Twist()
        move_cmd.linear.x = 0.3  # 降低速度
        move_cmd.angular.z = 5.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

    def sharp_right_turn(self):
        # 机器人右大转弯
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # 降低速度
        move_cmd.angular.z = -10.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

    def sharp_left_turn(self):
        # 机器人左大转弯
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # 降低速度
        move_cmd.angular.z = 10.0  # 降低角速度
        self.mecanum_pub.publish(move_cmd)

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
