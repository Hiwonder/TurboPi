#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import SetBool, Trigger, Empty

from speech import awake
from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetInt32

class VocalDetect(Node):
    """
    语音检测节点类，负责语音唤醒和语音识别
    Voice detection node class, responsible for voice wake-up and speech recognition
    """
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.running = True  # 运行标志 (running flag)

        # 声明参数 (declare parameters)
        self.declare_parameter('awake_method', 'xf')
        self.declare_parameter('mic_type', 'mic6_circle')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('enable_wakeup', True)
        self.declare_parameter('enable_setting', False)
        self.declare_parameter('awake_word', 'hello hi wonder')
        self.declare_parameter('mode', 1)

        self.awake_method = self.get_parameter('awake_method').value  # 唤醒方法 (wake-up method)
        mic_type = self.get_parameter('mic_type').value  # 麦克风类型 (microphone type)
        port = self.get_parameter('port').value  # 设备端口 (device port)
        awake_word = self.get_parameter('awake_word').value  # 唤醒词 (wake-up word)
        enable_setting = self.get_parameter('enable_setting').value  # 启用设置 (enable settings)
        self.enable_wakeup = self.get_parameter('enable_wakeup').value  # 启用唤醒 (enable wake-up)
        self.mode = int(self.get_parameter('mode').value)  # 工作模式 (working mode)

        # 初始化唤醒设备 (initialize wake-up device)
        if self.awake_method == 'xf':
            self.kws = awake.CircleMic(port, awake_word, mic_type, enable_setting)
        else:
            self.kws = awake.WonderEchoPro(port) 
        
        self.language = os.environ["ASR_LANGUAGE"]  # 获取语言环境 (get language environment)
        # 根据语言和唤醒方法初始化ASR (initialize ASR based on language and wake-up method)
        if self.awake_method == 'xf':
            if self.language == 'Chinese':
                self.asr = speech.RealTimeASR(log=self.get_logger())
            else:
                self.asr = speech.RealTimeOpenAIASR(log=self.get_logger())
                self.asr.update_session(model=asr_model, language='en')
        else:
            if self.language == 'Chinese':
                self.asr = speech.RealTimeASR(log=self.get_logger())
            else:
                self.asr = speech.RealTimeOpenAIASR(log=self.get_logger())
                self.asr.update_session(model=asr_model, language='en')
        
        # 创建发布者和服务 (create publishers and services)
        self.asr_pub = self.create_publisher(String, '~/asr_result', 1)  # 语音识别结果发布者 (ASR result publisher)
        self.wakeup_pub = self.create_publisher(Bool, '~/wakeup', 1)  # 唤醒状态发布者 (wake-up status publisher)
        self.awake_angle_pub = self.create_publisher(Int32, '~/angle', 1)  # 唤醒角度发布者 (wake-up angle publisher)
        self.create_service(SetInt32, '~/set_mode', self.set_mode_srv)  # 设置模式服务 (set mode service)
        self.create_service(SetBool, '~/enable_wakeup', self.enable_wakeup_srv)  # 启用唤醒服务 (enable wake-up service)

        # 启动发布回调线程 (start publisher callback thread)
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)  # 初始化完成服务 (initialization complete service)
        
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        """
        获取节点状态服务回调
        Node state service callback
        """
        return response

    def record(self, mode, angle=None):
        """
        录音并识别语音
        Record and recognize speech
        
        Args:
            mode (int): 工作模式 (working mode)
            angle (int, optional): 唤醒角度 (wake-up angle)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'asr...')
        if self.language == 'Chinese':
            asr_result = self.asr.asr(model=asr_model)  # 开启录音并识别 (start recording and recognition)
        else:
            asr_result = self.asr.asr()
        if asr_result: 
            speech.play_audio(dong_audio_path)  # 播放提示音 (play prompt sound)
            if self.awake_method == 'xf' and self.mode == 1: 
                msg = Int32()
                msg.data = int(angle)
                self.awake_angle_pub.publish(msg)  # 发布唤醒角度 (publish wake-up angle)
            asr_msg = String()
            asr_msg.data = asr_result
            self.asr_pub.publish(asr_msg)  # 发布语音识别结果 (publish ASR result)
            self.enable_wakeup = False
            self.get_logger().info('\033[1;32m%s\033[0m' % 'publish asr result:' + asr_result)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'no voice detect')
            speech.play_audio(dong_audio_path)
            if mode == 1:
                speech.play_audio(no_voice_audio_path)  # 播放无语音提示 (play no voice prompt)

    def pub_callback(self):
        """
        发布回调线程函数，处理唤醒和录音
        Publisher callback thread function, handling wake-up and recording
        """
        self.kws.start()  # 启动唤醒设备 (start wake-up device)
        while self.running:
            if self.enable_wakeup:
                if self.mode == 1:  # 唤醒模式 (wake-up mode)
                    result = self.kws.wakeup()
                    if result:
                        self.wakeup_pub.publish(Bool(data=True))  # 发布唤醒状态 (publish wake-up status)
                        speech.play_audio(wakeup_audio_path)  # 唤醒播放提示音 (play wake-up prompt)
                        self.record(self.mode, result)  # 录音并识别 (record and recognize)
                    else:
                        time.sleep(0.02)
                elif self.mode == 2:  # 直接录音模式 (direct recording mode)
                    self.record(self.mode)
                    self.mode = 1
                elif self.mode == 3:  # 另一种录音模式 (another recording mode)
                    self.record(self.mode)
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.02)
        rclpy.shutdown()

    def enable_wakeup_srv(self, request, response):
        """
        启用唤醒服务回调
        Enable wake-up service callback
        
        Args:
            request: 请求对象 (request object)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应 (service response)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % ('enable_wakeup'))
        self.kws.start()  # 启动唤醒设备 (start wake-up device)
        self.enable_wakeup = request.data  # 设置唤醒状态 (set wake-up status)
        response.success = True
        return response 

    def set_mode_srv(self, request, response):
        """
        设置模式服务回调
        Set mode service callback
        
        Args:
            request: 请求对象 (request object)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应 (service response)
        """
        self.get_logger().info(f'\033[1;32mset mode: {request.data}\033[0m')
        self.kws.start()  # 启动唤醒设备 (start wake-up device)
        self.mode = int(request.data)  # 设置工作模式 (set working mode)
        if self.mode != 1:
            self.enable_wakeup = True  # 非唤醒模式下启用唤醒 (enable wake-up in non-wake-up mode)
        response.success = True
        return response 

def main():
    """
    主函数
    Main function
    """
    node = VocalDetect('vocal_detect')  # 创建语音检测节点 (create voice detection node)
    try:
        rclpy.spin(node)  # 运行节点 (run node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown()  # 关闭ROS2 (shutdown ROS2)

if __name__ == "__main__":
    main()