import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DetectionResultSubscriber(Node):
    def __init__(self):
        super().__init__('detection_result_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/detection_result',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # 接收到消息后的处理逻辑
        detection_string = msg.data
        if not detection_string:
            self.get_logger().info('接收到空消息，未检测到物体')
            return  # 提前退出，不进行解析

        self.get_logger().info(f'接收到： {detection_string}')  # 输出原始信息

        try:
            detections = json.loads(detection_string)  # 将 JSON 字符串解析为 Python 列表
            for detection in detections:
                label = detection['label']
                confidence = detection['confidence']
                area = detection['area']

                # 输出解析后的信息
                self.get_logger().info(f'类别: {label}')
                self.get_logger().info(f'置信度: {confidence}')
                self.get_logger().info(f'面积: {area}')
                self.get_logger().info('---')

        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON 解析错误: {e}")  # 打印 JSON 解析错误信息
        except KeyError as e:
            self.get_logger().warn(f"KeyError: 缺少键 {e}")

def main(args=None):
    rclpy.init(args=args)
    detection_result_subscriber = DetectionResultSubscriber()
    rclpy.spin(detection_result_subscriber)
    detection_result_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
