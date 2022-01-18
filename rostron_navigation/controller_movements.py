from rclpy.action import ActionClient
from rostron_navigation.primitive.move_to import MoveTo
import rclpy
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.action import ActionServer
from rostron_navigation.joysticks import Joysticks
# from rostron_navigation.rostron_navigation.main import RobotNavigation

from rostron_utils.world import World
from rostron_interfaces.action import Behavior

import json

def create_move_to(x, y, theta):
    msg = Behavior.Goal()

    msg.name = "move_to"
    msg.params = json.dumps({"x": x, "y": y, "theta": theta})

    return msg

class Controller(Joysticks):
    tasks = []
    def __init__(self) -> None:
        super().__init__("Controller")
        self.client_ = ActionClient(World().node_, Behavior, f"robot_{id}/behavior")
        self.client_.wait_for_server()
        
        self.create_timer(0.16, self.update)

        self.declare_parameter('yellow', True)
        self.is_yellow = self.get_parameter('yellow').get_parameter_value().bool_value
        thisRobot = World().allies[0]
        World().init(self, self.is_yellow)
        self.client_.send_goal_async(create_move_to(0.0+self.player.x, 0.0+self.player.y, 0.0))
        

    def update(self):
        for t in self.tasks:
            t.update()