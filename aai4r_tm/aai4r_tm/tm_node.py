import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from threading import Lock
from time import time
import json
import uuid

class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('tm_node')

        # Subscriber for robot registration
        self.robot_registration_subscriber = self.create_subscription(
            String,
            '/aai4r/robot_registration',
            self.robot_registration_callback,
            10
        )
        self.robot_registration_subscriber  # prevent unused variable warning

        self.robot_registry = {}

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
            self.stt_subscriber_callback,
            10)
        self.stt_subscriber  # prevent unused variable warning

        # Publisher for Expression events
        self.expression_publisher = self.create_publisher(
            String,
            '/aai4r/expression_event', 10)

        # Publisher for menu selection
        self.selection_publisher = self.create_publisher(
            String,
            '/aai4r/selection_req',
            10
        )

        # Subscriber for menu selection
        self.selection_subscriber = self.create_subscription(
            String,
            '/aai4r/selection_res',
            self.selection_subscriber_callback,
            10
        )
        self.selection_subscriber

        self.navigation_publisher = self.create_publisher(
            String,
            '/aai4r/navigation_req',
            10
        )

        self.navigation_subscriber = self.create_subscription(
            String,
            '/aai4r/navigation_res',
            self.navigation_subscriber_callback,
            10
        )

        self.get_logger().info("Task Manager ready!")

        self.status = 0 # ready
        self.status_lock = Lock()
        self.timer = self.create_timer(3, self.cognitive_process)


    def set_status(self, new_status):
        with self.status_lock:
            self.status = new_status

    def get_status(self):
        return self.status

    def create_msg_id(self):
        return str(uuid.uuid1())

    def have_tablebot(self):
        for key in self.robot_registry.keys():
            robot = self.robot_registry[key]
            if robot[2] == 'tablebot':
                return True
        return False

    def get_tablebots(self):
        bots = []
        for key in self.robot_registry.keys():
            robot = self.robot_registry[key]
            if robot[2] == 'tablebot':
                bots.append(robot[0])
        return bots

    def get_deliverybots(self):
        bots = []
        for key in self.robot_registry.keys():
            robot = self.robot_registry[key]
            if robot[2] == 'deliverybot':
                bots.append(robot[0])
        return bots

    def robot_registration_callback(self, msg):
        msg_data = json.loads(msg.data)
        robot_id = msg_data['id']
        robot_type = msg_data['type']
        robot_role = msg_data['role']
        robot_loc = msg_data['loc']
        self.robot_registry[robot_id] = (robot_type, robot_role, robot_loc)
        self.get_logger().info('Robot Registered: {} {} {} {}'.format(robot_id, robot_type, robot_role, robot_loc))
        self.set_status(1) # now ready with at least one robot

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

    def send_navigation_req(self, target_pos, robot_id):
        msg = String()
        msg_data = {
            'msg_id': self.create_msg_id(),
            'robot_id': robot_id,
            'send_from': 'tm',
            'service': 'NAVIGATION_service',
            'msg_type': 'navigation_request',
            'payload': { 'cmd': 'request', 'target_pos': target_pos }
        }
        msg.data = json.dumps(msg_data)
        self.navigation_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def send_selection_req(self, selections, target_robot_id):
        msg = String()
        msg_data = {
            'msg_id': self.create_msg_id(),
            'robot_id': target_robot_id,
            'send_from': 'tm',
            'service': 'NAVIGATION_service',
            'msg_type': 'navigation_request',
            'payload': { 'cmd': 'request', 'selections': selections }
        }
        msg.data = json.dumps(msg_data)
        self.navigation_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def tts_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a TTS response message: "%s"' % msg.data)

    def stt_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a STT message: "%s"' % msg.data)

    def selection_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a selection message: "%s"' % msg.data)

    def navigation_subscriber_callback(self, msg):
        self.get_logger().info('\nGot a navigation response message: "%s"' % msg.data)

    def cognitive_process(self):
        status = self.get_status()
        if status == 0:
            return
        elif status == 1:
            tablebots = self.get_tablebots()
            if len(tablebots) > 0:
                self.set_status(2)
        elif status == 2:
            tablebots = self.get_tablebots()
            self.send_menu_items()
        elif status == 3:
            return


def main(args=None):
    rclpy.init(args=args)

    task_manager = TaskManagerNode()

    rclpy.spin(task_manager)

    task_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

