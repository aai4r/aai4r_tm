from logging import Logger

class TableBot(object):
    def __init__(self, robot_id) -> None:
        self.robot_id = robot_id

    def get_id(self):
        return self.robot_id

    def set_logger(self, logger) -> None:
        self.logger = logger

    def talk(self, msg, lang):
        self.logger.info("DeliveryBot({}): talks {} in language {}".format(self.robot_id, msg, lang))

    def express(self, expression_id):
        self.logger.info("DeliveryBot({}): expresses {}".format(self.robot_id, expression_id))

    def move(self, movement_id):
        self.logger.info("DeliveryBot({}): moves {}".format(self.robot_id, movement_id))

    def display(self, content):
        self.logger.info("DeliveryBot({}): displays {}".format(self.robot_id, content))

    def select(self, options):
        self.logger.info("DeliveryBot({}): selects {}".format(self.robot_id, options))
