#!/usr/bin/env python3
# encoding: utf-8
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorSpeedControl, MotorsSpeedControl, BuzzerState
from std_srvs.srv import Trigger
from enum import Enum


AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''


class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3


class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 0.2)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        # Initialize publishers
        self.publisher = self.create_publisher(MotorsSpeedControl, '/ros_robot_controller/set_motor_speeds', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.mecanum_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Initialize previous joystick inputs
        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))

        # Initialize mode and service
        self.mode = 0
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Joystick Controller started')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def val_map(self, value, in_min, in_max, out_min, out_max):
        """
        Maps a value from one range to another.
        :param value: The value to map
        :param in_min: The minimum of the input range
        :param in_max: The maximum of the input range
        :param out_min: The minimum of the output range
        :param out_max: The maximum of the output range
        :return: The mapped value
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def axes_callback(self, axes):
        twist = Twist()

        # Apply deadzone for joystick axes
        if abs(axes['lx']) < self.min_value:
            axes['lx'] = 0
        if abs(axes['ly']) < self.min_value:
            axes['ly'] = 0
        if abs(axes['rx']) < self.min_value:
            axes['rx'] = 0
        if abs(axes['ry']) < self.min_value:
            axes['ry'] = 0

        # Left joystick: Controls forward/backward (ly) and left/right (lx) movement
        twist.linear.x = self.val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)  # Forward/backward (Y-axis)
        twist.linear.y = self.val_map(axes['lx'], -1, 1, -self.max_linear, self.max_linear)  # Left/right (X-axis)

        # Right joystick: Controls rotation (rx) (for left/right self-rotation)
        twist.angular.z = self.val_map(axes['rx'], -1, 1, -self.max_angular, self.max_angular)  # Rotation (Z-axis)

        # Publish the twist message (for controlling the robot movement)
        self.mecanum_pub.publish(twist)

        # Calculate motor states for Mecanum wheels
        v1 = twist.linear.x + twist.linear.y + twist.angular.z
        v2 = twist.linear.x - twist.linear.y + twist.angular.z
        v3 = twist.linear.x - twist.linear.y - twist.angular.z
        v4 = twist.linear.x + twist.linear.y - twist.angular.z

        # Create MotorsSpeedControl message
        motor_speed_msg = MotorsSpeedControl()

        # Create and append MotorSpeedControl messages for each motor
        motor_speed_msg.data = [
            MotorSpeedControl(id=1, speed=-v1),  # Motor 1 (reverse speed)
            MotorSpeedControl(id=2, speed=v2),   # Motor 2
            MotorSpeedControl(id=3, speed=-v3),  # Motor 3 (reverse speed)
            MotorSpeedControl(id=4, speed=v4)    # Motor 4
        ]

        # Publish the motor speed control message
        self.servo_state_pub.publish(motor_speed_msg)



    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        self.axes_callback(axes)

        # Button callback for start button to activate buzzer
        if joy_msg.buttons[BUTTON_MAP.index('start')] == 1:
            self.start_callback(ButtonState.Pressed)

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # Play a short buzzer sound when the start button is pressed
            msg = BuzzerState()
            msg.freq = 2500  # Frequency in Hz
            msg.on_time = 0.05  # Duration of sound
            msg.off_time = 0.01  # Duration of silence
            msg.repeat = 1  # Repeat count
            self.buzzer_pub.publish(msg)

def main():
    node = JoystickController('joystick_control')
    rclpy.spin(node)

if __name__ == "__main__":
    main()