import uuid
import json

class Message(object):
    def __init__(self, send_from, send_to) -> None:
        self.msg_data = {
            'msg_id': self.create_id(),
            'send_from': send_from,
            'send_to': send_to
        }

    def get_msg_data(self) -> dict:
        return self.msg_data

    def get_msg(self) -> str:
        return json.dumps(self.msg_data)

    def create_id(self):
        return str(uuid.uuid1())


class REGISTRATION_REQ(Message):
    def __init__(self, send_from, send_to, role, location) -> None:
        super().__init__(send_from, send_to)
        self.msg_data['service'] = 'robot_registration'
        self.msg_data['msg_type'] = 'REGISTRATION_REQ'
        self.msg_data['payload'] = {
            'role': role,
            'loc': location
        }

class REGISTRATION_RES(Message):
    def __init__(self, send_from, send_to) -> None:
        super().__init__(send_from, send_to)
        self.msg_data['service'] = 'robot_registration'
        self.msg_data['msg_type'] = 'REGISTRATION_RES'

class ORDER_REQ(Message):
    def __init__(self, send_from, send_to, order) -> None:
        super().__init__(send_from, send_to)
        self.order_id = self.create_id()
        self.msg_data['service'] = 'order_processing'
        self.msg_data['msg_type'] = 'ORDER_REQ'
        self.msg_data['payload'] = {
            'order_id': self.order_id,
            'order': order
        }

    def get_order_id(self) -> str:
        return self.order_id


class ORDER_RES(Message):
    def __init__(self, send_from, send_to, order_id) -> None:
        super().__init__(send_from, send_to)
        self.msg_data['service'] = 'order_processing'
        self.msg_data['msg_type'] = 'ORDER_RES'
        self.msg_data['payload'] = {
            'order_id': order_id
        }
