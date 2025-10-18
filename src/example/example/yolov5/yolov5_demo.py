import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # 创建控制发布器
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 订阅检测结果话题
        self.create_subscription(String, '/detection_result', self.detection_callback, 10)

        # 定义运动控制速度
        self.speed = 0.7
        self.turn_speed = 0.5

    def detection_callback(self, msg):
        # 解析检测结果
        detected_labels = msg.data.split(', ')  # 获取标签和分数列表
        move_cmd = Twist()

        for label in detected_labels:
            if 'green' in label:  # 如果检测到“green”标签，前进
                move_cmd.linear.x = self.speed
                move_cmd.angular.z = 0.0
            elif 'red' in label:  # 如果检测到“red”标签，后退
                move_cmd.linear.x = -self.speed
                move_cmd.angular.z = 0.0
            elif 'right' in label:  # 如果检测到“right”标签，右转
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = -self.turn_speed
            elif 'left' in label:  # 如果检测到“left”标签，左转
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = self.turn_speed
            else:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0  # 停止

        # 发布控制命令
        self.publisher.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorControl()
    rclpy.spin(motor_control)
    motor_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
