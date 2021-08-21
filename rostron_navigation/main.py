from rostron_navigation.primitive.move_to import MoveToStrategie
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

        # Change init to node !
        World().init(self)

        # Create all action and service for the navigation system.
        self.get_logger().info(
            f"Init actions and service for robot {self.id}")

        self._action_server = ActionServer(
            self, Behavior, f"robot_{self.id}/behavior", self.new_behavior)

        self.get_logger().info(
            "Finish to init all action and service")

    def new_behavior(self, msg_handle: ServerGoalHandle):
        msg: Behavior.Goal = msg_handle.request
        self.get_logger().info(msg.params)
        self._logger.info(msg.name)
        params = json.loads(msg.params)

        self._logger.info(f"{params['x']}, {params['y']}, {params['theta']}")

        robot = MoveToStrategie(self.id)
        robot.move_to((params['x'], params['y']), params['theta'])

        msg_handle.succeed()

        result = Behavior.Result()
        return result

        pass


def main():
    rclpy.init()
    node = RobotNavigation()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
