from aai4r_tm.logging import Logger

class Logger4ROS2(Logger):
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

    def info(self, msg):
        self.logger.info(msg)

    def debug(self, msg):
        self.logger.debug(msg)
    