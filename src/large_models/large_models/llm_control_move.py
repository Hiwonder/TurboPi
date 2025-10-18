#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2025/02/20

import re
import time
import math
import rclpy
import threading
from speech import speech
from rclpy.node import Node
from large_models.config import *
from geometry_msgs.msg import Twist

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from large_models_msgs.srv import SetModel,SetString
from ros_robot_controller_msgs.msg import RGBState,RGBStates, SetPWMServoState, PWMServoState

if os.environ["ASR_LANGUAGE"] == 'Chinese': 
    PROMPT = '''
# 角色
你是一款智能视觉麦轮小车，需要根据输入的内容，生成对应的json指令。

##要求与限制
1.根据输入的动作内容，在动作函数库中找到对应的指令，并输出对应的指令。
2.为动作序列编织一句精炼（10至30字）、风趣且变化无穷的反馈信息，让交流过程妙趣横生。
3.直接输出json结果，不要分析，不要输出多余内容。
4.格式：{'action':['xx', “xx”], 'response':'xx'}

##结构要求##：
- `"action"`键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]。 
- `"response"`键则配以精心构思的简短回复，完美贴合上述字数与风格要求。 

## 动作函数库
- 前进一秒：forward 
- 后退一秒：back
- 向左转：turn_left
- 向右转：turn_right
- 飘移：drift
- 点头：nod
- 摇头：shake_head
- 红灯：(255,0,0)
- 绿灯：(0,255,0)
- 蓝灯：(0,0,255)
- 关灯：(0,0,0)

### 任务示例：
输入:亮红灯。
输出:{"action":["(255,0,0)"], "response":"红灯亮起，小心驶得万年船，是不是有点像电影里的紧急刹车？"}
输入:先前进两秒。
输出:{'action":["forward", "forward"], "response":"双蹄飞奔，勇往直前！"}
输入:先向左转，再向右转。
输出:{"action":["turn_left", "turn_right"], "response":"左转变右转，看我操控自如！"}
'''
else:
    PROMPT='''
# Role
You are a smart visual wheeled car. You need to generate corresponding json instructions based on the input content.
## Requirements and limitations
1. Find the corresponding instruction in the action function library according to the input action content, and output the corresponding instruction.
2. Weave a concise (10 to 30 words), witty and ever-changing feedback message for the action sequence, making the communication process interesting.
3. Directly output the json result, do not analyze, do not output extra content.
4. Format: {'action':['xx', “xx”], 'response':'xx'}
## Structure requirements##：
- The "action" key carries an array of function name strings arranged in the order of execution. When the corresponding action function cannot be found, action outputs [].
- The "response" key is paired with a carefully crafted short reply, perfectly fitting the above word count and style requirements.
## Action function library
- Move forward for one second: forward
- Move backward for one second: back
- Turn left: turn_left
- Turn right: turn_right
- Drift: drift
- Nod: nod
- Shake head: shake_head
- Red light: (255,0,0)
- Green light: (0,255,0)
- Blue light: (0,0,255)
- Turn off the light: (0,0,0)
### Task example:
Input: Turn on the red light.
Output: {"action":["(255,0,0)"], "response":"The red light is on. Be careful to drive for ten thousand years. Is it a bit like the emergency brake in the movie?"}
Input: First move forward for two seconds.
Output: {'action':["forward", "forward"], "response":"Double hooves gallop forward!"}
Input: First turn left, then turn right.
Output: {"action":["turn_left", "turn_right"], "response":"Left turn becomes right turn, look at me controlling freely!"}
    '''

class LLMControlMove(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
               
        self.action = []
        self.interrupt = False
        self.llm_result = ''
        self.running = True
        self.result = ''
        self.action_finish = False
        self.play_audio_finish = False

        self.mecanum_pub = self.create_publisher(Twist, 'cmd_vel', 1)  # 底盘控制(chassis control) 
        self.pwm_pub = self.create_publisher(SetPWMServoState,'ros_robot_controller/pwm_servo/set_state',10)    
        self.sonar_rgb_pub = self.create_publisher(RGBStates, 'sonar_controller/set_rgb', 10)   
        self.rgb_pub = self.create_publisher(RGBStates, 'ros_robot_controller/set_rgb', 10)    
        
        timer_cb_group = ReentrantCallbackGroup()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)           
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)      
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        
        
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)
        
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()

        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
                               
        
    
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        msg = SetModel.Request()
        msg.model = llm_model
        msg.model_type = 'llm'
        msg.api_key = api_key 
        msg.base_url = base_url
        
        self.set_model_client.call_async(msg)
        
        msg = SetString.Request()
        msg.data = PROMPT
        self.set_prompt_client.call_async(msg)

        self.mecanum_pub.publish(Twist())     
        speech.play_audio(start_audio_path)  
        
        threading.Thread(target=self.process, daemon=True).start()
        
        self.create_service(Trigger, '~/init_finish', self.get_node_state)       
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start') 
        self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
    
    def wakeup_callback(self, msg):
        if self.llm_result:
            self.get_logger().info('wakeup interrupt')
            self.interrupt = msg.data
    
   
    def llm_result_callback(self, msg):
        self.get_logger().info(msg.data)       
        self.llm_result = msg.data 
        
    def play_audio_finish_callback(self, msg):
        msg = SetBool.Request()
        msg.data = True
        self.awake_client.call_async(msg)
        self.play_audio_finish = msg.data
    
    def process(self):
        while self.running:           
            if self.llm_result != '':
                if 'action' in self.llm_result:
                    self.result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}')+1])
                    if 'action' in self.result:
                        action_list = self.result['action']
                    if 'response' in self.result:
                        response = self.result['response']
                    else:
                        response = self.result
                else:
                    time.sleep(0.02)
                response_msg = String()
                response_msg.data = response
                self.tts_text_pub.publish(response_msg)
                
                for a in action_list: 
                    if self.interrupt:
                            self.get_logger().info('interrupt')
                            break 
                    if a == "forward":
                        twist = Twist()
                        twist.linear.x = 0.3
                        self.mecanum_pub.publish(twist)
                        time.sleep(1)  
                        self.mecanum_pub.publish(Twist())                      
                    elif a == "back":
                        twist = Twist()
                        twist.linear.x = -0.5
                        self.mecanum_pub.publish(twist)
                        time.sleep(1)  
                        self.mecanum_pub.publish(Twist()) 
                    elif a == "turn_left":
                        twist = Twist()
                        twist.angular.z = 3.0
                        self.mecanum_pub.publish(twist)
                        time.sleep(3.15)  
                        self.mecanum_pub.publish(Twist()) 
                    elif a == "turn_right":
                        twist = Twist()
                        twist.angular.z = -3.0
                        self.mecanum_pub.publish(twist)
                        time.sleep(3.15)  
                        self.mecanum_pub.publish(Twist()) 
                    elif a == "drift":
                        twist = Twist()
                        twist.linear.x = 0.4
                        twist.angular.z = 5.0
                        self.mecanum_pub.publish(twist)
                        time.sleep(3)  
                        self.mecanum_pub.publish(Twist()) 
                    elif a == "nod":
                        self.pwm_controller([1, 1800])
                        time.sleep(0.3)
                        self.pwm_controller([1, 1200])
                        time.sleep(0.3)
                        self.pwm_controller([1, 1500])
                        time.sleep(0.3)
                        
                    elif a == "shake_head":
                        self.pwm_controller([2, 1200])
                        time.sleep(0.3)
                        self.pwm_controller([2, 1800])
                        time.sleep(0.3)
                        self.pwm_controller([2, 1500])
                        time.sleep(0.3)                                              
                    # checking light                   
                    elif a.startswith("(") and a.endswith(")") and a.count(",") == 2:
                        numbers = a.strip("()").split(",")
                        r, g, b = map(int, numbers)                                              
                        self.sonar_rgb_controller(r, g, b)                      
        
                    else: 
                        self.get_logger().warn("{} is not find".format(a))

                    if self.interrupt:
                            self.interrupt = False
                            self.mecanum_pub.publish(Twist())
                            break
                
                self.action_finish = True 
                self.llm_result = ''
            else: 
                time.sleep(0.01)
            if self.play_audio_finish and self.action_finish:
                self.play_audio_finish = False
                self.action_finish = False
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
    
    def sonar_rgb_controller(self, r, g, b):
        def create_led(index):
            led = RGBState()
            led.index = index
            led.red = r
            led.green = g
            led.blue = b
            return led
        # 发布给 sonar_rgb_pub
        msg1 = RGBStates()
        msg1.states = [create_led(0), create_led(1)]
        self.sonar_rgb_pub.publish(msg1)
    
        # 发布给 rgb_pub
        msg2 = RGBStates()
        msg2.states = [create_led(1), create_led(2)]
        self.rgb_pub.publish(msg2)
        
        
def main():
    node = LLMControlMove('llm_control_move')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
