import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from aai4r_tm.logging4ros2 import Logger4ROS2

from aai4r_tm.deliverybot import DeliveryBot

import json
import uuid

class DeliveryBotNode(Node):

    def __init__(self):
        self.robot_id = 'deliverybot_node_' + (str(uuid.uuid1())).replace('-', '_')

        super().__init__(self.robot_id)

        self.bot = DeliveryBot(self.robot_id)
        self.bot.set_logger(Logger4ROS2(self))

        # Publisher for Navigation responses
        self.navigation_publisher = self.create_publisher(
            String,
            '/aai4r/navigation_res',
            10)

        # Subscriber for Navigation requests 
        self.navigation_subscriber = self.create_subscription(
            String,
            '/aai4r/navigation_req', 
            self.navigation_subscriber_callback,
            10)
        self.navigation_subscriber  # prevent unused variable warning

        self.send_registration_req()

    def send_registration_req(self):
        msg = String()
        msg_data = {
            'id': self.robot_id,
            'type': 'servbot',
            'role': 'deliverybot',
            'loc': 'poi01'
        }
        msg.data = json.dumps(msg_data)
        self.registration_publisher.publish(msg)
        self.get_logger().info("Registration message published: {}".format(msg.data))


    def navigation_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a Nagivation request: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        payload = msg_data['payload']
        self.bot.talk(payload['speech_text'], payload['lang'])

    # listens to the bot for the response to the selection request
    def selection_listener(self, selection):
        self.get_logger().info('\nPublishing a selection result: "%s"' % selection)
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


def main(args=None):
    rclpy.init(args=args)

    node = TablebotNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

