#!/usr/bin/env python3
# encoding: utf-8
# @Author: liang
# @Date: 2025/3/7
import cv2
import json
import time
import queue
import rclpy
import threading
import subprocess  
import sdk.fps as fps
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel
from cv_bridge import CvBridge

language = os.environ["ASR_LANGUAGE"]
if language == 'Chinese':
    PROMPT = '''
你作为智能管家机器人，可以实时识别图像并理解动物，检测动物是食肉动物还是食草动物，
response 中输出相关信息，你发现有食肉动物和食草动物在一起的话，就发出警报声吧。
格式：
{
    "type": "detect",
    "object": "xxx",
    "action": ["xxx"],
    "response": "xxxxx"
}


 - 示例：
     如果指令为：  
     "你的前面有哪些动物呢？如果你发现有食肉动物和食草动物在一起的话，就发出警报声吧"  
     你返回：## 输出格式（请仅输出以下内容，不要说任何多余的话)
     {
      "type": "detect",
      "object": "动物类型",
      "action": ["alert()"],
      "response": "小心前面有什么食肉动物和是什么动物在一起很危险！提醒什么动物要远离！"
     }
'''
else:
    PROMPT = '''
You are a smart housekeeper robot. You can identify animals in real time and understand whether they are carnivores or herbivores.
If you find carnivores and herbivores together, please sound the alarm.
frormat:
{
    "type": "detect",
    "object": "xxx",
    "action": ["xxx"],
    "response": "xxxxx"
}

 - Example:    
        If the instruction is:
        "What animals are in front of you? If you find carnivores and herbivores together, please sound the alarm."
        You return: ## Output format (please only output the following content, do not say anything extra)
        {           
        "type": "detect",
        "object": "animal type",
        "action": ["alert()"],  
        "response": "Be careful, there are carnivores and herbivores together in front of you, which is dangerous! Remind what animals to stay away!"
        }
        '''

class VLLMTrack(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.vllm_result = ''

        self.running = True
        self.data = []
        self.start_track = False
        timer_cb_group = ReentrantCallbackGroup()
        self.client = speech.OpenAIAPI(api_key, base_url)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.bridge = CvBridge()

        self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1,
                                 callback_group=timer_cb_group)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)

        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()

        msg = SetModel.Request()
        msg.model = vllm_model
        msg.model_type = 'vllm'
        if os.environ['ASR_LANGUAGE'] == 'Chinese':
            msg.model = stepfun_vllm_model
            msg.api_key = stepfun_api_key
            msg.base_url = stepfun_base_url
        else:
            msg.model = vllm_model
            msg.api_key = vllm_api_key
            msg.base_url = vllm_base_url
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
        if not self.vllm_result or len(self.vllm_result) < 5:  # 检查结果是否为空或太短
            self.get_logger().warn("vllm_result 为空或太短，忽略.")
            self.vllm_result = ''  # 丢弃不可靠的结果
        else:
            time.sleep(0.02)  # 等待 2 秒钟，确保音频播放完成

    def play_audio_finish_callback(self, msg):
        if msg.data:
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)

    def process(self):
        cv2.namedWindow('image', 0)
        while self.running:
            image = self.image_queue.get(block=True)
            if self.vllm_result:
                try:
                    response = json.loads(self.vllm_result.strip())  # 移除可能存在的空白字符
                    if "action" in response and isinstance(response['action'], list):
                        self.get_logger().info(f"Action list: {response['action']}")  # 打印 action 列表
                        # 首先检查是否需要播放警报音
                        if "alert()" in response['action']:  # 使用 'alert()' 作为触发警报的指令
                            speech.play_audio(alert_audio_path)
                            response['action'].remove("alert()") # 移除，避免重复播放
                    else:
                        self.get_logger().warn("JSON 响应中缺少 'action' 字段或不是列表，跳过动作执行。")

                    msg = String()
                    msg.data = response['response']
                    time.sleep(2)
                    self.tts_text_pub.publish(msg)
                    self.get_logger().info(response['response'])

                except (ValueError, TypeError) as e:
                    self.start_track = False
                    msg = String()
                    msg.data = str(e)  # 将异常转换为字符串
                    self.tts_text_pub.publish(msg)
                    self.get_logger().error(f"JSON 解析错误: {str(e)}")  # 使用 error 级别来记录解析失败

                self.vllm_result = ''  # 清除结果，*在* 处理或处理失败后
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)

            self.fps.update()
            self.fps.show_fps(image)
            cv2.imshow('image', image)
            cv2.waitKey(1)

        cv2.destroyAllWindows()
    def image_callback(self, ros_image):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
            if self.image_queue.full():
                self.image_queue.get()
            self.image_queue.put(rgb_image)
        except Exception as e:
            self.get_logger().error(f"转换图像时出错: {e}")
            speech.speak_text("图像转换出错啦!") # 语音提示

def main():
    node = VLLMTrack('vllm_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
