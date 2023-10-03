from rclpy.action import ActionClient
from rclpy.node import Node

from pick_place_interfaces.action import PickPlace


class MainActionClient(Node):
    def __init__(self):
        super().__init__('main_action_client')
        self._action_client = ActionClient(self, PickPlace, 'perform_pick_place')

    def send_goal(self, perform_pick_place):
        goal_msg = PickPlace.Goal()
        goal_msg.perform_pick_place = perform_pick_place

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)
