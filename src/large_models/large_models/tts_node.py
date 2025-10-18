#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import time
import rclpy
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import String, Bool

from speech import speech
from large_models.config import *

class TTSNode(Node):
    """
    文本转语音节点，负责将文本转换为语音并播放
    Text-to-Speech node, responsible for converting text to speech and playing it
    """
    def __init__(self, name):
        """
        初始化TTS节点
        Initialize the TTS node
        
        Args:
            name: 节点名称 (node name)
        """
        rclpy.init()
        super().__init__(name)
        
        self.text = None  # 待转换的文本 (text to be converted)
        speech.set_volume(100)  # 设置音量为100% (set volume to 100%)
        self.language = os.environ["ASR_LANGUAGE"]  # 从环境变量获取语言设置 (get language setting from environment variable)
        if self.language == 'Chinese':
            self.tts = speech.RealTimeTTS(log=self.get_logger())  # 中文TTS引擎 (Chinese TTS engine)
        else:
            self.tts = speech.RealTimeOpenAITTS(log=self.get_logger())  # 英文TTS引擎 (English TTS engine)
       
        # 创建发布者和订阅者 (create publisher and subscriber)
        self.play_finish_pub = self.create_publisher(Bool, '~/play_finish', 1)  # 播放完成发布者 (play finish publisher)
        self.create_subscription(String, '~/tts_text', self.tts_callback, 1)  # 订阅文本消息 (subscribe to text message)

        # 启动TTS处理线程 (start TTS processing thread)
        threading.Thread(target=self.tts_process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)  # 创建初始化完成服务 (create initialization finish service)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')  # 打印启动信息 (print start information)

    def get_node_state(self, request, response):
        """
        获取节点状态服务回调
        Node state service callback
        
        Args:
            request: 请求 (request)
            response: 响应 (response)
        
        Returns:
            response: 响应对象 (response object)
        """
        return response

    def tts_callback(self, msg):
        """
        文本消息回调函数
        Text message callback function
        
        Args:
            msg: 包含要转换为语音的文本的消息 (message containing text to be converted to speech)
        """
        # self.get_logger().info(msg.data)
        self.text = msg.data  # 更新待处理的文本 (update text to be processed)

    def tts_process(self):
        """
        TTS处理线程函数，持续监听并处理文本转语音请求
        TTS processing thread function, continuously monitors and processes text-to-speech requests
        """
        while True:
            if self.text is not None:  # 如果有待处理的文本 (if there is text to be processed)
                if self.text == '':
                    speech.play_audio(no_voice_audio_path)  # 播放无语音提示音 (play no voice prompt sound)
                else:
                    self.tts.tts(self.text, model=tts_model, voice=voice_model)  # 执行文本转语音 (perform text-to-speech)
                self.text = None  # 清空已处理的文本 (clear processed text)
                msg = Bool()
                msg.data = True
                self.play_finish_pub.publish(msg)  # 发布播放完成消息 (publish play finish message)
            else:
                time.sleep(0.01)  # 短暂休眠以减少CPU占用 (short sleep to reduce CPU usage)

def main():
    node = TTSNode('tts_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown() 

if __name__ == "__main__":
    main()