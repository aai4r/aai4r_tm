from aai4r_tm.logging import Logger

class DeliveryBot(object):
    def __init__(self, robot_id) -> None:
        self.robot_id = robot_id

    def set_logger(self, logger) -> None:
        self.logger = logger

    def talk(self, msg):
        self.logger.info("DeliveryBot({}): talks {}".format(self.robot_id, msg))

    def move_to(self, waypoint_id):
        self.logger.info("DeliveryBot({}): moves to {}".format(self.robot_id, waypoint_id))

