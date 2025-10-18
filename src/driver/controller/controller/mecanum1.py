import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorsSpeedControl, MotorSpeedControl

class DifferentialDriveController(Node):
    def __init__(self, a=67, b=59, wheel_diameter=65):
        super().__init__('differential_drive_controller')
        
        # 麦轮的参数
        self.a = a  # mm (前后轮之间的距离)
        self.b = b  # mm (左右轮之间的距离)
        self.wheel_diameter = wheel_diameter  # mm
        self.wheel_radius = wheel_diameter / 2.0  # mm (轮子的半径)

        # 目标速度初始化
        self.target_linear_velocity = 0.0  # 目标线速度 (m/s)
        self.target_angular_velocity = 0.0  # 目标角速度 (rad/s)

        # 创建发布者来控制电机的速度
        self.motor_speed_pub = self.create_publisher(MotorsSpeedControl, '/set_motor_speeds', 10)
        
        # 创建订阅者来接收线速度和角速度
        self.create_subscription(Twist, '/controller/cmd_vel', self.set_cmd_vel, 10)  # 控制目标线速度和角速度

        # 设置默认线速度和角速度
        self.set_cmd_vel(Twist())  # 默认线速度和角速度为0

    def set_cmd_vel(self, msg):
        """ 设置目标线速度和角速度 """
        self.target_linear_velocity = msg.linear.x
        self.target_angular_velocity = msg.angular.z
        self.update_motor_speeds()

    def update_motor_speeds(self):
        """ 计算并更新四个电机的速度 """
        v_x = self.target_linear_velocity  # 目标前进线速度
        w_z = self.target_angular_velocity  # 目标角速度

        # 根据麦轮的运动学公式，计算每个电机的线速度
        v1 = (v_x - (self.a + self.b) * w_z) / self.wheel_radius
        v2 = (v_x + (self.a + self.b) * w_z) / self.wheel_radius
        v3 = (v_x + (self.a + self.b) * w_z) / self.wheel_radius
        v4 = (v_x - (self.a + self.b) * w_z) / self.wheel_radius

        # 将速度限制到电机的最大范围（假设最大速度为 [-100, 100]）
        v1 = max(min(v1, 100), -100)
        v2 = max(min(v2, 100), -100)
        v3 = max(min(v3, 100), -100)
        v4 = max(min(v4, 100), -100)

        # 创建 MotorsSpeedControl 消息并发布控制命令
        motor_speeds = MotorsSpeedControl()

        # 使用 MotorSpeedControl 对象填充数据
        motor_speeds.data = [
            MotorSpeedControl(id=1, speed=int(v1)),
            MotorSpeedControl(id=2, speed=int(v2)),
            MotorSpeedControl(id=3, speed=int(v3)),
            MotorSpeedControl(id=4, speed=int(v4))
        ]
        
        # 发布电机速度控制消息
        self.motor_speed_pub.publish(motor_speeds)

def main():
    rclpy.init()
    controller = DifferentialDriveController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
