import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorSpeedControl, MotorsSpeedControl  

class MecanumChassis(Node):
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065, max_linear_speed=1.0, max_angular_speed=1.0):
        super().__init__('mecanum_chassis')

        # 麦克纳姆轮的物理参数
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter
        self.max_linear_speed = max_linear_speed  # 最大线性速度 (m/s)
        self.max_angular_speed = max_angular_speed  # 最大角速度 (rad/s)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(MotorsSpeedControl, '/ros_robot_controller/set_motor_speeds', 10)

    def speed_convert(self, speed, max_speed):
        """
        将速度从 m/s 转换为电机速度，并映射到 -100 到 100 范围。
        :param speed: 速度（m/s）
        :param max_speed: 最大速度（m/s）
        :return: 电机控制速度 (-100 到 100)
        """
        # 按比例将速度转换为电机控制速度
        motor_speed = (speed / max_speed) * 100
        motor_speed = max(-100, min(100, motor_speed))  # 限制在 -100 到 100 之间
        return motor_speed
    # def speed_convert(self, speed, max_speed, angular_speed_factor=10.0):
    #     """
    #     将速度从 m/s 转换为电机速度，并映射到 -100 到 100 范围。
    #     :param speed: 速度（m/s）或角速度（rad/s）
    #     :param max_speed: 最大速度（m/s）或角速度
    #     :param angular_speed_factor: 角速度的比例因子，控制角速度的映射范围
    #     :return: 电机控制速度 (-100 到 100)
    #     """
    #     # 对角速度的映射进行调整
    #     if speed == 0:
    #         motor_speed = 0
    #     else:
    #         if abs(speed) <= max_speed:
    #             motor_speed = (speed / max_speed) * 100
    #         else:
    #             motor_speed = angular_speed_factor * speed
    #             motor_speed = max(-100, min(100, motor_speed))  # 限制在 -100 到 100 之间
        
    #     return motor_speed


    def cmd_vel_callback(self, msg: Twist):
        """
        订阅/cmd_vel话题，获取线性速度和角速度，计算并发布四个电机的速度。
        :param msg: 订阅的Twist消息
        """
        linear_x = msg.linear.x  # 线性速度（前后）
        linear_y = msg.linear.y  # 线性速度（左右）
        angular_z = msg.angular.z  # 角速度（绕z轴）

        # 计算四个电机的速度
        motor_msg = self.set_velocity(linear_x, linear_y, angular_z)

        # 发布电机速度
        self.publisher.publish(motor_msg)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        根据线性速度和角速度计算每个电机的控制速度
        :param linear_x: x方向线性速度（m/s）
        :param linear_y: y方向线性速度（m/s）
        :param angular_z: 角速度（rad/s）
        :return: MotorsSpeedControl 消息，包含四个电机的速度信息
        """
        # 计算四个电机的速度
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)

        # 映射电机速度到 -100 到 100 范围
        motor_speeds = [self.speed_convert(v, self.max_linear_speed) for v in [-motor1, motor3, -motor2, motor4]]

        msg = MotorsSpeedControl()
        msg.data = []
        for i in range(4):
            motor_msg = MotorSpeedControl()
            motor_msg.id = i + 1  # 电机编号
            motor_msg.speed = float(motor_speeds[i])  # 电机速度
            msg.data.append(motor_msg)

        return msg
    # def set_velocity(self, linear_x, linear_y, angular_z):
    #     """
    #     根据线性速度和角速度计算每个电机的控制速度
    #     :param linear_x: x方向线性速度（m/s）
    #     :param linear_y: y方向线性速度（m/s）
    #     :param angular_z: 角速度（rad/s）
    #     :return: MotorsSpeedControl 消息，包含四个电机的速度信息
    #     """
    #     motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
    #     motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
    #     motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
    #     motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)

    #     # 使用调整过的speed_convert
    #     motor_speeds = [self.speed_convert(v, self.max_linear_speed, angular_speed_factor=10.0) for v in [-motor1, motor2, -motor3, motor4]]

    #     msg = MotorsSpeedControl()
    #     msg.data = []

    #     for i in range(4):
    #         motor_msg = MotorSpeedControl()
    #         motor_msg.id = i + 1  # 电机编号
    #         motor_msg.speed = float(motor_speeds[i])  # 电机速度
    #         msg.data.append(motor_msg)

    #     return msg

def main(args=None):
    # 初始化rclpy库
    rclpy.init(args=args)

    # 创建MecanumChassis节点
    mecanum_chassis = MecanumChassis()

    # 保持节点运行
    rclpy.spin(mecanum_chassis)

    # 销毁节点
    mecanum_chassis.destroy_node()

    # 关闭rclpy库
    rclpy.shutdown()


if __name__ == '__main__':
    main()
