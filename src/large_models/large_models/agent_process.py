#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import queue
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool, Empty

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel, SetContent

class AgentProcess(Node):
    """
    智能体处理节点类，负责处理语音识别结果并调用大模型生成回答
    Agent processing node class, responsible for processing speech recognition results and calling large models to generate answers
    """
    def __init__(self, name):
        """
        初始化智能体处理节点
        Initialize the agent processing node
        
        Args:
            name: 节点名称 (node name)
        """
        rclpy.init()
        super().__init__(name)
        
        self.declare_parameter('camera_topic', 'image_raw')  # 声明相机话题参数 (declare camera topic parameter)
        camera_topic = self.get_parameter('camera_topic').value  # 获取相机话题 (get camera topic)

        self.prompt = ''  # 提示词 (prompt)
        self.model = llm_model  # 大语言模型 (large language model)
        self.chat_text = ''  # 聊天文本 (chat text)
        self.model_type = 'llm'  # 模型类型 (model type)
        self.start_record_chat = False  # 开始记录聊天标志 (start recording chat flag)
        self.bridge = CvBridge()  # 图像转换桥接器 (image conversion bridge)
        self.image_queue = queue.Queue(maxsize=2)  # 图像队列，最大容量为2 (image queue with max capacity of 2)
        self.client = speech.OpenAIAPI(api_key, base_url)  # 初始化OpenAI API客户端 (initialize OpenAI API client)
        
        # 创建发布者和订阅者 (create publisher and subscribers)
        self.result_pub = self.create_publisher(String, '~/result', 1)  # 结果发布者 (result publisher)
        self.create_subscription(String, 'vocal_detect/asr_result', self.asr_callback, 1)  # 语音识别结果订阅 (ASR result subscription)
        self.create_subscription(Image, camera_topic, self.image_callback, 1)  # 摄像头订阅 (subscribe to the camera)
        
        # 创建服务 (create services)
        self.create_service(SetModel, '~/set_model', self.set_model_srv)  # 设置模型服务 (set model service)
        self.create_service(SetString, '~/set_prompt', self.set_prompt_srv)  # 设置提示词服务 (set prompt service)
        self.create_service(SetContent, '~/set_llm_content', self.set_llm_content_srv)  # 设置LLM内容服务 (set LLM content service)
        self.create_service(SetContent, '~/set_vllm_content', self.set_vllm_content_srv)  # 设置VLLM内容服务 (set VLLM content service)

        self.create_service(SetBool, '~/record_chat', self.record_chat)  # 记录聊天服务 (record chat service)
        self.create_service(Trigger, '~/get_chat', self.get_chat)  # 获取聊天服务 (get chat service)
        self.create_service(Empty, '~/clear_chat', self.clear_chat)  # 清除聊天服务 (clear chat service)

        self.create_service(Empty, '~/init_finish', self.get_node_state)  # 初始化完成服务 (initialization complete service)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')  # 打印启动信息 (print start information)

    def get_node_state(self, request, response):
        """
        获取节点状态服务回调
        Node state service callback
        
        Args:
            request: 请求对象 (request object)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应 (service response)
        """
        return response

    def record_chat(self, request, response):
        """
        记录聊天服务回调
        Record chat service callback
        
        Args:
            request: 请求对象，包含是否记录聊天的标志 (request object containing flag for recording chat)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应 (service response)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'record chat')
        self.start_record_chat = request.data  # 设置记录聊天标志 (set recording chat flag)
        response.success = True
        return response

    def get_chat(self, request, response):
        """
        获取聊天服务回调
        Get chat service callback
        
        Args:
            request: 请求对象 (request object)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应，包含聊天文本 (service response containing chat text)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'get chat')
        response.message = self.chat_text.rstrip(",")  # 返回去除末尾逗号的聊天文本 (return chat text with trailing comma removed)
        response.success = True
        return response

    def clear_chat(self, request, response):
        """
        清除聊天服务回调
        Clear chat service callback
        
        Args:
            request: 请求对象 (request object)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应 (service response)
        """
        self.get_logger().info('\033[1;32m%s\033[0m' % 'clear chat')
        self.chat_text = ''  # 清空聊天文本 (clear chat text)
        self.record_chat = False  # 停止记录聊天 (stop recording chat)
        return response

    def asr_callback(self, msg):
        """
        语音识别结果回调函数
        ASR result callback function
        
        Args:
            msg: 包含语音识别结果的消息 (message containing ASR result)
        """
        # self.get_logger().info(msg.data)
        # 将识别结果传给智能体让他来回答 (pass recognition result to agent for answering)
        if msg.data != '':
            self.get_logger().info('\033[1;32m%s\033[0m' % 'thinking...')
            if self.start_record_chat:
                self.chat_text += msg.data + ','  # 添加到聊天记录 (add to chat record)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'record chat:' + self.chat_text)
            res = ''
            if self.model_type == 'llm':
                res = self.client.llm(msg.data, self.prompt, model=self.model)  # 调用语言模型 (call language model)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish llm result:' + str(res))
            elif self.model_type == 'vllm':
                image = self.image_queue.get(block=True)  # 从队列获取图像 (get image from queue)
                res = self.client.vllm(msg.data, image, prompt=self.prompt, model=self.model)  # 调用视觉语言模型 (call vision language model)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish vllm result:' + str(res))
            msg = String()
            msg.data = res
            self.result_pub.publish(msg)  # 发布结果 (publish result)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'asr result none')

    def image_callback(self, ros_image):
        """
        图像回调函数
        Image callback function
        
        Args:
            ros_image: ROS图像消息 (ROS image message)
        """
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")  # 转换为OpenCV图像 (convert to OpenCV image)
        bgr_image = np.array(cv_image, dtype=np.uint8)  # 转换为NumPy数组 (convert to NumPy array)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像 (if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列 (put the image into the queue)
        self.image_queue.put(bgr_image)

    def set_model_srv(self, request, response):
        """
        设置模型服务回调
        Set model service callback
        
        Args:
            request: 请求对象，包含模型信息 (request object containing model information)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应 (service response)
        """
        # 设置调用哪个模型 (set which model to call)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set model')
        self.model = request.model  # 设置模型 (set model)
        self.model_type = request.model_type  # 设置模型类型 (set model type)
        self.client = speech.OpenAIAPI(request.api_key, request.base_url)  # 更新API客户端 (update API client)
        response.success = True
        return response

    def set_prompt_srv(self, request, response):
        """
        设置提示词服务回调
        Set prompt service callback
        
        Args:
            request: 请求对象，包含提示词 (request object containing prompt)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应 (service response)
        """
        # 设置大模型的prompt (set prompt for large model)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set prompt')
        self.prompt = request.data  # 更新提示词 (update prompt)
        response.success = True
        return response

    def set_llm_content_srv(self, request, response):
        """
        设置LLM内容服务回调
        Set LLM content service callback
        
        Args:
            request: 请求对象，包含查询和提示词 (request object containing query and prompt)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应，包含LLM回答 (service response containing LLM answer)
        """
        # 输入文本传给智能体让他来回答 (input text is passed to agent for answering)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'thinking...')
        client = speech.OpenAIAPI(request.api_key, request.base_url)  # 创建临时客户端 (create temporary client)
        response.message = client.llm(request.query, request.prompt, model=request.model)  # 调用语言模型 (call language model)
        response.success = True
        return response

    def set_vllm_content_srv(self, request, response):
        """
        设置VLLM内容服务回调
        Set VLLM content service callback
        
        Args:
            request: 请求对象，包含查询、图像和提示词 (request object containing query, image and prompt)
            response: 响应对象 (response object)
            
        Returns:
            response: 服务响应，包含VLLM回答 (service response containing VLLM answer)
        """
        # 输入提示词和文本，视觉智能体返回回答 (input prompt and text, vision agent returns answer)
        if request.image.data:
            self.get_logger().info(f'receive image')  # 接收到图像 (received image)
            image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")  # 转换图像 (convert image)
        else:
            image = self.image_queue.get(block=True)  # 从队列获取图像 (get image from queue)
        client = speech.OpenAIAPI(request.api_key, request.base_url)  # 创建临时客户端 (create temporary client)
        res = client.vllm(request.query, image, prompt=request.prompt, model=request.model)  # 调用视觉语言模型 (call vision language model)
        response.message = res
        response.success = True
        return response

def main():
    """
    主函数
    Main function
    """
    node = AgentProcess('agent_process')  # 创建智能体处理节点 (create agent process node)
    try:
        rclpy.spin(node)  # 运行节点 (run node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown()  # 关闭ROS2 (shutdown ROS2)

if __name__ == "__main__":
    main()