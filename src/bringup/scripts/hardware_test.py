#!/usr/bin/env python3
# encoding: utf-8

import sys
import time
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState,MotorSpeedControl, MotorsSpeedControl 


class HardwareTestNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.servo_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 10)
        self.motor_pub = self.create_publisher(MotorsSpeedControl, 'ros_robot_controller/set_motor_speeds', 10)

        self.pwm_servo(1,1800)
        time.sleep(0.3)
        self.pwm_servo(1,1500)
        time.sleep(0.3)
        self.pwm_servo(1,1200)
        time.sleep(0.3)
        self.pwm_servo(1,1500)
        time.sleep(0.3)
        self.pwm_servo(2,1200)
        time.sleep(0.3)
        self.pwm_servo(2,1500)
        time.sleep(0.3)
        self.pwm_servo(2,1800)
        time.sleep(0.3)
        self.pwm_servo(2,1500)
        time.sleep(0.3)
        
        for i in range(1, 5):
            self.motor_control(i, 45)
            time.sleep(0.3)
            self.motor_control(i, 0.0)
            time.sleep(0.3)
        sys.exit(0)

    def motor_control(self, id, speed):
        msg = MotorsSpeedControl()
        motor_msg = MotorSpeedControl()
        motor_msg.id = id  # 电机编号
        motor_msg.speed = float(speed)  # 电机速度
        msg.data.append(motor_msg)
        self.motor_pub.publish(msg)

    def pwm_servo(self, id, position):
        msg = SetPWMServoState()
        state_x = PWMServoState()
        state_x.id = [id]
        state_x.position = [int(position)]
        msg.state = [state_x]
        msg.duration = 0.02
        self.servo_pub.publish(msg)

    
def main():
    rclpy.init()
    node = HardwareTestNode('hardware_test')
    try:
        rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
