#!/usr/bin/env python3
# encoding: utf-8
# @Author: liang
# @Date: 2025/3/7
import cv2
import json
import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from std_srvs.srv import SetBool, Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel
from ros_robot_controller_msgs.msg import BuzzerState
from geometry_msgs.msg import Twist

language = os.environ["ASR_LANGUAGE"]
if language == 'Chinese':
    PROMPT = '''
你作为智能机器人，可以实时识别图像，并在response 中输出相关信息.

我会给你一句话，从我的话中提取距离和动作，将获取的距离(单位是厘米，要转化成毫米)和动作填充进distance_one和action，"action"键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]。

##动作函数库
* 往前走：move_forward()
* 左转：turn_left()
* 停止运动：move_backward()
* 右转：turn_right()
* 左平移：move_left()
* 右平移：move_right()
* 加速：move_forward_fast()
* 减速：move_forward_slow()
* 叫一下:buzzer()

格式：
{
    "type": "detect",
    "object": "障碍物",
    "action": ["xx", "xx"],
    "distance_one": [300, 200],
    "response": "xxx"
}

##示例
输入：往前走如果前方30cm有障碍物开始减速，前方20cm有障碍物就左转
输出：{
        "type": "detect",
        "object": "障碍物",
        "action": ["move_forward()", "move_forward_slow()", "turn_right()"],
        "distance_one": [300, 200],
        "response": "已完成避障"
    }
'''
else:
    PROMPT = '''
You are an intelligent robot that can recognize images in real time and output relevant information in the response.
I will give you a sentence, extract the distance and action from my words, and fill the obtained distance (in centimeters, converted to millimeters) and action into distance_one and action. The "action" key carries an array of function name strings arranged in execution order. When there is no corresponding action function, the action outputs [].
## Action function library
* Move forward: move_forward()
* Turn left: turn_left()        
* Stop moving: move_backward()
* Turn right: turn_right()
* Move left: move_left()
* Move right: move_right()
* Speed up: move_forward_fast()
* Slow down: move_forward_slow()
* Make a sound: buzzer()

- Format:
{
    "type": "detect",
    "object": "obstacle",
    "action": ["xx", "xx"],
    "distance_one": [300, 200],
    "response": "xxx"
}

## Example
Input: Move forward. If there is an obstacle 30cm ahead, slow down. If there is an obstacle 20cm ahead, turn left.
Output: {
        "type": "detect",
        "object": "obstacle",
        "action": ["move_forward()", "move_forward_slow()", "turn_right()"],
        "distance_one": [300, 200],
        "response": "Obstacle avoidance completed"
    }   
'''

class VLLMTrack(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.vllm_result = ''
        self.running = True
        self.data = []
        self.start_track = False
        timer_cb_group = ReentrantCallbackGroup()
        self.client = speech.OpenAIAPI(api_key, base_url)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.buzzer_publisher = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 10)
        self.mecanum_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1, callback_group=timer_cb_group)

        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()

        self.distance = 0
        self.distance_subscription = self.create_subscription(Int32,'sonar_controller/get_distance',self.distance_callback,1)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
        self.action_index = 0
        self.actions = []
        self.distances = []
        self.response_text = ""
        self.current_action = None
        self.action_start_time = None  # 记录动作开始时间
        self.is_moving_forward = False # 用于标记是否正在前进
        self.all_actions_completed = False  # 标记所有动作是否完成
        self.initial_forward_executed = False # 初始前进是否执行过
        self.tts_speaking = False # Flag to indicate if TTS is currently speaking


    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()


        msg = SetModel.Request()
        msg.model_type = 'vllm'
        if language == 'Chinese':
            msg.model = stepfun_vllm_model
            msg.api_key = stepfun_api_key
            msg.base_url = stepfun_base_url
        else:
            msg.api_key = vllm_api_key
            msg.base_url = vllm_base_url
            msg.model = vllm_model
        self.send_request(self.set_model_client, msg)

        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_prompt_client, msg)

        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def vllm_result_callback(self, msg):
        self.vllm_result = msg.data
        #self.get_logger().info(f"接收到的 vllm_result: {self.vllm_result}")

        try:
            '''
            json_string = self.vllm_result.strip()

            if json_string.startswith("```json"):
                json_string = json_string[6:]
            if json_string.endswith("```"):
                json_string = json_string[:-3]

            if json_string.startswith("{") and json_string.endswith("}"):
                pass
            else:
                json_string = json_string[1:-1]
            response = json.loads(json_string.strip())
            '''

            response = eval(self.vllm_result[self.vllm_result.find('{'):self.vllm_result.find('}')+1])

            if "action" in response and isinstance(response['action'], list):
                self.actions = response['action']
                self.distances = response.get('distance_one', [])
                self.response_text = response['response']
                self.action_index = 0 # reset
                self.stop_motion() # 停止所有运动
                self.is_moving_forward = False  # 重置前进状态
                self.all_actions_completed = False # 重置标志位
                self.initial_forward_executed = False # 重置初始前进标志

            else:
                self.get_logger().warn("JSON 响应中缺少 'action' 字段或不是列表，跳过动作执行。")
                self.actions = []
                self.distances = []
                self.response_text = ""
            msg = String()
            msg.data = response['response']
            self.tts_text_pub.publish(msg)
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"JSON 解析错误: {str(e)}")
            self.get_logger().error(f"原始字符串：{self.vllm_result}")
            self.actions = []
            self.distances = []
            self.response_text = ""
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)

    def play_audio_finish_callback(self, msg):
        if msg.data:
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)
            self.tts_speaking = False  # TTS finished speaking

    def buzzer(self,count = 1):
        msg = BuzzerState()
        msg.freq = 1000
        msg.on_time = 0.2
        msg.off_time = 0.9
        msg.repeat = count
        self.buzzer_publisher.publish(msg)

    def process(self):
        while self.running:
            # 只有有action指令时才会执行
            if self.vllm_result and self.actions:
                current_distance = self.distances[self.action_index] if self.action_index < len(self.distances) else None
                current_action = self.actions[self.action_index]

                # 先判断是否是move_backward, 如果是，不需要距离直接执行。
                if current_action == "move_backward()":
                    self.stop_motion()
                    self.execute_action_for_one_second(current_action)
                    self.action_index += 1
                    if self.action_index >= len(self.actions):
                        self.all_actions_completed = True
                        self.all_actions_completed_callback()
                    continue

                # 如果当前动作是 move_forward
                if current_action == "move_forward()":
                    # 如果当前不在前进，则启动前进
                    if not self.is_moving_forward:
                        self.start_moving_forward()
                        self.is_moving_forward = True
                    # 如果当前已经在前进，则根据距离判断是否停止
                    elif current_distance is not None:
                        if self.distance <= current_distance:
                            self.stop_motion()  # 停止前进
                            self.is_moving_forward = False
                            self.get_logger().info(f"距离 {self.distance} 小于等于 {current_distance}, 停止并执行下一个动作")
                            self.action_index += 1  # 移动到下一个动作
                            if self.action_index >= len(self.actions):
                                self.all_actions_completed = True
                                self.all_actions_completed_callback()
                            continue  # 执行下一个动作

                # 如果当前动作不是 move_forward
                else:
                    self.stop_motion()
                    self.execute_action_for_one_second(current_action)
                    self.action_index += 1
                    if self.action_index >= len(self.actions):
                        self.all_actions_completed = True
                        self.all_actions_completed_callback()
                    continue

    def all_actions_completed_callback(self):
        """所有动作完成后执行的回调函数."""
        if not self.tts_speaking:  # 确保TTS没有在说话
            self.stop_motion()  # 停止所有运动
            self.get_logger().info("finish")
            self.get_logger().info(self.response_text)
            self.tts_speaking = True  # Set TTS speaking flag

            self.vllm_result = ''  # 清空结果
            self.all_actions_completed = False  # 重置所有动作已完成标志
            self.action_index = 0  # 重置action index，以便下次正常运行
            self.initial_forward_executed = False

            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)

    def execute_action_for_one_second(self, action_str):
        """执行指定动作 2 秒钟."""
        self.execute_action(action_str)
        self.get_logger().info(f"开始执行动作 {action_str} 2 秒钟.")
        time.sleep(0.6)
        self.stop_motion()



    def start_moving_forward(self):
        """开始前进."""
        twist = Twist()
        twist.linear.x = 0.4  # 设置前进速度
        self.mecanum_pub.publish(twist)
        self.is_moving_forward = True
        self.get_logger().info("开始前进")

    def stop_motion(self):
        """停止所有运动."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.mecanum_pub.publish(twist)
        self.get_logger().info("停止运动")
        self.is_moving_forward = False
        self.current_action = None

    def execute_action(self, action_str):
        twist = Twist()
        if action_str == "move_forward()":
            twist.linear.x = 0.4
            self.get_logger().info("前进")
        elif action_str == "move_backward()":
            twist.linear.x = 0.0
            self.get_logger().info("后退")
        elif action_str == "turn_left()":
            twist.angular.z = 4.0
            self.get_logger().info("左转")
        elif action_str == "turn_right()":
            twist.angular.z = -4.0
            self.get_logger().info("右转")
        elif action_str == "move_left()":
            twist.linear.y = 2.5
            self.get_logger().info("左移")
        elif action_str == "move_right()":
            twist.linear.y = -2.5
            self.get_logger().info("右移")
        elif action_str == "move_forward_fast()":
            twist.linear.x = 0.7
            self.get_logger().info("快速前进")
        elif action_str == "move_forward_slow()":
            twist.linear.x = 0.3
            self.get_logger().info("缓慢前进")
        elif action_str == "buzzer()":
            self.buzzer()
            self.get_logger().info('蜂鸣')
        else:
            self.get_logger().warn(f"未知动作: {action_str}")
            return

        self.mecanum_pub.publish(twist)
        self.get_logger().info(f"正在执行动作: {action_str}")
        self.current_action = action_str


    def distance_callback(self, msg):
        self.distance = msg.data
        # self.get_logger().info(f"当前距离: {self.distance} mm")

def main():
    node = VLLMTrack('vllm_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
