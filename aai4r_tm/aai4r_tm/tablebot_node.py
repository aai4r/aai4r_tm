import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from threading import Lock
from threading import Thread
from aai4r_tm.tablebot import TableBot
from aai4r_tm.logging4ros2 import Logger4ROS2
from aai4r_tm.msgs import REGISTRATION_REQ
from aai4r_tm.msgs import REGISTRATION_RES
from aai4r_tm.msgs import ORDER_REQ
from aai4r_tm.msgs import ORDER_RES

import json
import uuid
from time import time

class TablebotNode(Node):

    def __init__(self):
        id = 'tablebot_' + (str(uuid.uuid1())).replace('-', '_')

        super().__init__(id)

        self.task_manager_id = None
        self.menu_items = None
        self.current_order = None

        self.bot = TableBot(id)
        self.bot.set_logger(Logger4ROS2(self))

        self.registration_publisher = self.create_publisher(String, '/aai4r/robot_registration', 10)
        self.registration_subscriber = self.create_subscription(String, '/aai4r/robot_registration', self.registration_subscriber_callback, 10)
        self.registration_subscriber

        # Publisher for TTS responses
        self.tts_publisher = self.create_publisher(String, '/aai4r/tts_res', 10)
        self.tts_subscriber = self.create_subscription(String, '/aai4r/tts_req', self.tts_subscriber_callback, 10)
        self.tts_subscriber  # prevent unused variable warning

        # Pulisher for STT results
        self.stt_publisher = self.create_publisher(String, '/aai4r/tts_res', 10)

        # Subscriber for Expression events
        self.expression_subscriber = self.create_subscription(String, '/aai4r/expression_event', self.expression_subscriber_callback, 10)
        self.expression_subscriber  # prevent unused variable warning

        # Subscriber for menu display requests
        self.display_subscriber = self.create_subscription(String, '/aai4r/display_req', self.display_subscriber_callback, 10)
        self.display_subscriber  # prevent unused variable warning

        # Publisher for menu orders
        self.order_publisher = self.create_publisher(String, '/aai4r/order', 10)
        self.order_subscriber = self.create_subscription(String, '/aai4r/order', self.order_subscriber_callback, 10)
        self.order_subscriber  # prevent unused variable warning

        # Subscriber for menu items notification
        self.menu_subscriber = self.create_subscription(String, '/aai4r/menu', self.menu_subscriber_callback, 10)
        self.menu_subscriber  # prevent unused variable warning

        self.navigation_subscriber = self.create_subscription(String, "/aai4r/navigation", self.navigation_subscriber_callback, 10)

        self.status = 0
        self.status_lock = Lock()

        self.timer = self.create_timer(1, self.cognitive_process)

        self.get_logger().info('tablebot_node initialized.')


    def get_id(self):
        return self.bot.get_id()

    def get_status(self):
        return self.status

    def set_status(self, new_status):
        with self.status_lock:
            self.status = new_status

    # COGNITIVE PROCESS =================================================================
    def cognitive_process(self):
        status = self.get_status()
        if status == 0:
            self.send_registration_req()
            self.get_logger().info('tablebot_node sent a registration request.')
            self.set_status(1)
        elif status == 1:
            if self.task_manager_id is None:
                self.send_registration_req()
                self.get_logger().info('retrying a tablebot_node registration request.')
            self.set_status(2)
        elif status == 2:
            if self.menu_items is not None:
                self.set_status(3)
        elif status == 3:
            if self.current_order is None:
                self.get_logger().info("I am ready to get menu orders...")
                self.start_to_wait_for_user_order()

    def start_to_wait_for_user_order(self):
        x = Thread(target=self.wait_for_user_order, args=())
        x.start()

    def wait_for_user_order(self):
        # wait for user input
        ok = input('Please type any key to signal that you have completed an order.\n')
        self.get_logger().info('Got a user input.')
        order = {'김치찌개':2, '비빔밥': 1} # TODO: 실제 주문을 받아야 함
        self.send_order(order)
        self.get_logger().info('Sent out an order: {}'.format(order))

    def start_to_wait_for_unload_completed(self):
        x = Thread(target=self.wait_for_unload_completed, args=())
        x.start()

    def wait_for_unload_completed(self):
        # wait for user input
        ok = input('Please type any key to signal that you unloaded trays.\n')
        self.get_logger().info('Got a user input.')
        self.send_unload_completed()
        self.get_logger().info('Sent out an unload completed message.')

    # Robot Registration and Response Processing ===========================================
    def send_registration_req(self):
        msg = String()
        msg_data = REGISTRATION_REQ(self.get_id(), '', 'tablebot', 'table01').get_msg_data()
        msg.data = json.dumps(msg_data)
        self.registration_publisher.publish(msg)
        self.get_logger().info("Registration message published: {}".format(msg.data))
        self.set_status(1)

    def registration_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a registration response: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        if msg_data['msg_type'] == 'REGISTRATION_RES' and msg_data['send_to'] == self.get_id():
            self.get_logger().info("   not a message to me.")
            self.set_task_manager(msg_data['send_from'])

    def set_task_manager(self, id):
        self.task_manager_id = id
        self.get_logger().info('\nNow, I know the manager: "%s"' % id)

    # Robot Registration and Response Processing ------------------------------------------------
    def menu_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a menu notification: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        # the empty send_to field means the message is broadcasted to every robot
        if msg_data['msg_type'] == 'MENU_ITEMS_NOTI' and msg_data['send_to'] == '':
            payload = msg_data['payload']
            self.menu_items = payload['menu_items']
            self.get_logger().info("Today's menu items: {}".format(self.menu_items))

    # send out an order to task manager
    def send_order(self, order):
        self.get_logger().info('\nPublishing an order: "%s"' % order)
        order_req = ORDER_REQ(self.get_id(), self.task_manager_id, order)
        self.current_order = order_req.get_order_id()
        msg = String()
        msg.data = order_req.get_msg()
        self.order_publisher.publish(msg)

    def order_subscriber_callback(self, msg):
        self.get_logger().info('\nGot an order arrival notification: "%s"' % msg.data)
        msg_data = json.loads(msg.data)
        # Order arrival processing
        if msg_data['msg_type'] == 'ORDER_ARRIVED' and msg_data['send_to'] == self.get_id():
            if msg_data['payload']['order_id'] == self.current_order:
                self.get_logger().info("   Order {} has arrived!".format(self.current_order))
                self.bot.talk("Order Arrived. Please unload the trays!")
                self.start_to_wait_for_unload_completed()
            else:
                self.get_logger().info("  Order {} is not mine.".format(msg_data['payload']['order_id']))
        else:
            self.get_logger().info("     not my message...")

    # send out an order to task manager
    def send_unload_completed(self):
        self.get_logger().info('\nPublishing a notification of unload completion.')
        msg = String()
        msg_data = {
            'msg_id': self.create_msg_id(),
            'robot_id': self.bot.get_id(),
            'send_from': self.bot.get_id(),
            'senf_to': self.task_manager_id,
            'service': 'order_service',
            'msg_type': 'unload_completed',
        }
        msg.data = json.dumps(msg_data)
        self.order_publisher.publish(msg)

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

    def create_msg_id(self):
        return str(uuid.uuid1())

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

