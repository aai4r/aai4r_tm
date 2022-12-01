import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json
import uuid

class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('tm_node')

        # Publisher for TTS requests
        self.tts_publisher = self.create_publisher(
            String,
            '/aai4r/tts_req', 10)

        # Subscriber for TTS reponses 
        self.tts_subscriber = self.create_subscription(
            String,
            '/aai4r/tts_res', 
            self.tts_subscriber_callback, 10)
        self.tts_subscriber  # prevent unused variable warning

        # Subscriber for STT
        self.stt_subscriber = self.create_subscription(
            String,
            '/aai4r/stt_event', 
            self.stt_subscriber_callback, 10)
        self.stt_subscriber  # prevent unused variable warning

        # Publisher for Expression events
        self.expression_publisher = self.create_publisher(
            String,
            '/aai4r/expression_event', 10)

        self.timer = self.create_timer(3, self.timer_callback)

    def create_msg_id(self):
        return str(uuid.uuid1())

    def send_tts_req(self, text, robot_id):
        msg = String()
        msg_data = {
            'msg_id': self.create_msg_id(),
            'robot_id': robot_id,
            'send_from': 'tm',
            'service': 'TTS_service',
            'msg_type': 'homebot_tts_req',
            'payload': { 'cmd': 'request', 'lang':'ko', 'speech_text': text }
        }
        msg.data = json.dumps(msg_data)
        self.tts_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def send_expression_req(self, expression, robot_id):
        msg = String()
        msg_data = {
            'msg_id': self.create_msg_id(),
            'robot_id': robot_id,
            'send_from': 'tm',
            'service': 'EXPRESSION_service',
            'msg_type': 'hombot_expression_event',
            'payload': { 'cmd': 'event', 'expression': expression }
        }
        msg.data = json.dumps(msg_data)
        self.expression_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def tts_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a TTS response message: "%s"' % msg.data)

    def stt_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a STT message: "%s"' % msg.data)

    def timer_callback(self):
        val = input('1. TTS@homebot, 2. TTS@kitchenbot, 3. Expression@homebot, 4. Expression@kitchenbot, Enter Number: ')
        val = int(val)
        if val == 1:
            self.send_tts_req('안녕하세요.', 'homebot')
        elif val == 2:
            self.send_tts_req('안녕하세요.', 'kitchenbot')
        elif val == 3:
            self.send_expression_req('EX_03', 'homebot')
        elif val == 4:
            self.send_expression_req('EX_03', 'kitchenbot')
        else:
            print("NO....")


def main(args=None):
    rclpy.init(args=args)

    task_manager = TaskManagerNode()

    rclpy.spin(task_manager)

    task_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

