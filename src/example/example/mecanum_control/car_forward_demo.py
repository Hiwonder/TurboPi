#!/usr/bin/env python3
# encoding: utf-8

# 小车前进

import rclpy
import signal
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CarMoveDemo(Node):
    def __init__(self):
        super().__init__('car_forward') 
        signal.signal(signal.SIGINT, self.stop)  
        self.mecanum_pub = self.create_publisher(Twist, 'cmd_vel', 1)  # 底盘控制
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):    
        twist = Twist()
        twist.linear.x = 0.5  # 设置线速度
        self.mecanum_pub.publish(twist)
        self.get_logger().info(f"Published twist: {twist.linear.x}")
        
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