import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from threading import Lock
from threading import Thread
from aai4r_tm.tablebot import TableBot
from aai4r_tm.logging4ros2 import Logger4ROS2

import json
import uuid
from time import time

class TablebotNode(Node):

    def __init__(self):
        self.robot_id = 'tablebot_node_' + (str(uuid.uuid1())).replace('-', '_')

        super().__init__(self.robot_id)

        self.bot = TableBot(self.robot_id)
        self.bot.set_logger(Logger4ROS2(self))

        self.registration_publisher = self.create_publisher(
            String,
            '/aai4r/robot_registration',
            10
        )

        # Publisher for TTS responses
        self.tts_publisher = self.create_publisher(
            String,
            '/aai4r/tts_res',
            10)

        # Subscriber for TTS requests 
        self.tts_subscriber = self.create_subscription(
            String,
            '/aai4r/tts_req', 
            self.tts_subscriber_callback,
            10)
        self.tts_subscriber  # prevent unused variable warning

        # Pulisher for STT results
        self.stt_publisher = self.create_publisher(
            String,
            '/aai4r/tts_res',
            10)

        # Subscriber for Expression events
        self.expression_subscriber = self.create_subscription(
            String,
            '/aai4r/expression_event', 
            self.expression_subscriber_callback,
            10)
        self.expression_subscriber  # prevent unused variable warning

        # Subscriber for menu display requests
        self.display_subscriber = self.create_subscription(
            String,
            '/aai4r/display_req',
            self.display_subscriber_callback,
            10
        )
        self.display_subscriber  # prevent unused variable warning

        # Subscriber for menu selection
        self.selection_subscription = self.create_subscription(
            String,
            '/aai4r/selection_req',
            self.selection_subscription_callback,
            10
        )
        self.selection_subscription  # prevent unused variable warning

        # Publisher for menu selection
        self.selection_publisher = self.create_publisher(
            String,
            '/aai4r/selection_res',
            10
        )

        # Subscriber for menu items notification
        self.menu_subscriber = self.create_subscription(
            String,
            '/aai4r/menu',
            self.menu_subscriber_callback,
            10
        )
        self.menu_subscriber  # prevent unused variable warning

        # 0: initial
        # 1: robot registrated
        # 2: menu items received
        # 3: guests are seated
        # 4: menu ordered
        # 5: menu served
        self.status = 0
        self.status_lock = Lock()

        self.timer = self.create_timer(1, self.cognitive_process)

        self.send_registration_req()


    def set_status(self, new_status):
        with self.status_lock:
            self.status = new_status

    def get_status(self):
        return self.status

    def menu_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a menu notification: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        payload = msg_data['payload']
        self.menu_items = payload['menu_items']
        self.get_logger().info("Today's menu items: {}".format(self.menu_items))
        self.set_status(2)

    def send_registration_req(self):
        msg = String()
        msg_data = {
            'id': self.robot_id,
            'type': 'homebot',
            'role': 'tablebot',
            'loc': 'table01'
        }
        msg.data = json.dumps(msg_data)
        self.registration_publisher.publish(msg)
        self.get_logger().info("Registration message published: {}".format(msg.data))
        self.set_status(1)

    def tts_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a TTS request: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        payload = msg_data['payload']
        self.bot.talk(payload['speech_text'], payload['lang'])

    def expression_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a Expression request: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        payload = msg_data['payload']
        self.bot.express(payload['expression'])

    def display_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a display request: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        payload = msg_data['payload']
        self.bot.display(payload['display_text'])

    def selection_subscription_callback(self, msg):
        self.get_logger().info('\nGot a selection request: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        payload = msg_data['payload']
        self.bot.select(payload['options'])

    # listens to the bot for the response to the selection request
    def send_selections(self, selections):
        self.get_logger().info('\nPublishing a set of selections: "%s"' % selections)
        msg = String()
        msg_data = {
            'msg_id': self.create_msg_id(),
            'robot_id': self.bot.get_id(),
            'send_from': self.bot.get_id(),
            'service': 'selection_service',
            'msg_type': 'selection_response',
            'payload': { 'cmd': 'selection_response', 'selection':selection }
        }
        self.selection_publisher.publish(msg)

    def create_msg_id(self):
        return str(uuid.uuid1())

    def cognitive_process(self):
        # 0: initial
        # 1: robot registrated
        # 2: menu items received
        # 3: guests are seated
        # 4: menu ordered
        # 5: menu served
        status = self.get_status()
        if status == 0:
            return
        elif status == 1:
            return
        elif status == 2:
            self.set_next_status(5, 3)
        elif status == 3:
            selections = self.bot.select(self.menu_items)
            self.send_selections(selections)
            self.set_status(4)
    
    def set_next_status(self, seconds, next_status):
        x = Thread(target=self.thread_function, args=(self, seconds, next_status, ))
        x.start()

    def thread_function(self, seconds, next_status):
        time.sleep(seconds)
        self.set_status(next_status)

def main(args=None):
    rclpy.init(args=args)

    node = TablebotNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

