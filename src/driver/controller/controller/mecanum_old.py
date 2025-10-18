import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorSpeedControl, MotorsSpeedControl  

class MecanumChassis(Node):
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        super().__init__('mecanum_chassis')

        # 麦克纳姆轮的物理参数
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(MotorsSpeedControl, '/ros_robot_controller/set_motor_speeds', 10)

    def speed_convert(self, speed):
        """
        将速度从米每秒(m/s)转换为电机速度
        :param speed: 速度（m/s）
        :return: 电机控制速度
        """
        motor_speed = speed / (math.pi * self.wheel_diameter)
        return motor_speed

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
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)

        # 转换为电机速度，并返回计算结果
        v_s = [self.speed_convert(v) for v in [-motor1, motor2, -motor3, motor4]]

        msg = MotorsSpeedControl()
        msg.data = []

        for i in range(4):
            motor_msg = MotorSpeedControl()
            motor_msg.id = i + 1  #
            motor_msg.speed = float(v_s[i])  
            msg.data.append(motor_msg)

        return msg


def main(args=None):
    rclpy.init(args=args)
    mecanum_chassis = MecanumChassis()
    rclpy.spin(mecanum_chassis)
    mecanum_chassis.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
