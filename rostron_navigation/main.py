from rostron_navigation.primitive.move_to import MoveTo
import rclpy
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.action import ActionServer

from rostron_utils.world import World
from rostron_interfaces.action import Behavior

import json

class RobotNavigation(Node):
    def __init__(self) -> None:
        super().__init__("navigation")
        self.declare_parameter("id", 0)
        self.id = self.get_parameter("id").get_parameter_value().integer_value

        # TODO : Change init to node !
        World().init(self)

        self.get_logger().info(f"Init navigation for robot {self.id}")

        self._action_server = ActionServer(
            self, Behavior, f"robot_{self.id}/behavior", self.new_behavior)

        self.get_logger().info("Finish to init")

    def new_behavior(self, msg_handle: ServerGoalHandle):
        msg: Behavior.Goal = msg_handle.request

        params = json.loads(msg.params)

        self._logger.info(f"{params['x']}, {params['y']}, {params['theta']}")
        
        # Move this in the behavior timer.
        robot = MoveTo(self.id)
        finish = False
        while not(finish):
            finish = robot.move_to((params['x'], params['y'], params['theta']))

        msg_handle.succeed()

        result = Behavior.Result()

        return result


def main():
    rclpy.init()
    node = RobotNavigation()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
