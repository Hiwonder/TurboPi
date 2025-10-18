#!/usr/bin/env python3
# encoding: utf-8

# 小车前后左右运动

import rclpy
import signal
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CarMoveDemo(Node):
    def __init__(self):
        super().__init__('car_move') 
        signal.signal(signal.SIGINT, self.stop)  
        self.mecanum_pub = self.create_publisher(Twist, 'cmd_vel', 1)  # 底盘控制
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.state = 0  # 当前状态
        self.twist = Twist()  # 运动命令

    def timer_callback(self):
        if self.state == 0:
            self.twist.linear.x = 0.5  # 向前
            self.twist.linear.y = 0.0
        elif self.state == 1:
            self.twist.linear.x = -0.5  # 向后
            self.twist.linear.y = 0.0
        elif self.state == 2:
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.5  # 向左
        elif self.state == 3:
            self.twist.linear.x = 0.0
            self.twist.linear.y = -0.5  # 向右

        self.mecanum_pub.publish(self.twist)
        self.get_logger().info(f"Published twist: linear.x = {self.twist.linear.x}, linear.y = {self.twist.linear.y}")
        
        # 更新状态
        self.state = (self.state + 1) % 4  # 循环状态

    
        
    def stop(self, signum, frame):
        self.mecanum_pub.publish(Twist())
        rclpy.shutdown()

def main():
    rclpy.init()  
    node = CarMoveDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.mecanum_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()