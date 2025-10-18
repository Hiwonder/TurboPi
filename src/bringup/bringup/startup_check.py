#!/usr/bin/env python3
# encoding: utf-8

import rclpy
import time
from ros_robot_controller_msgs.msg import BuzzerState,SetPWMServoState, PWMServoState
from rclpy.node import Node

class StartupCheckNode(Node):
    def __init__(self):
        super().__init__('startup_check_node')
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.pwm_pub = self.create_publisher(SetPWMServoState,'ros_robot_controller/pwm_servo/set_state',10)
        self.timer = self.create_timer(1.0, self.publish_control)
        self.get_logger().info('StartupCheckNode initialized') 

    def publish_control(self):
        
        # controller buzzer
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
        self.get_logger().info('Buzzer state published')
        self.get_logger().info(f'Buzzer state published: {msg}')
        
        # controller pwm
        self.pwm_controller([1, 1500], [2, 1500])       
        rclpy.shutdown()
        
    def pwm_controller(self, *position_data):
        pwm_list = []
        msg = SetPWMServoState()
        msg.duration = 0.2
        for position in position_data:
            pos = PWMServoState()
            pos.id = [position[0]]
            pos.position = [int(position[1])]
            pwm_list.append(pos)
        msg.state = pwm_list
        self.pwm_pub.publish(msg)

def main():
    rclpy.init()
    node = StartupCheckNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

