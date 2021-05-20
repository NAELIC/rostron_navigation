import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rostron_interfaces.action import MoveTo

from .primitive.move_to import MoveToPrimitive


class Navigation(Node):

    def __init__(self, id: int) -> None:
        super().__init__('navigation')

        self.id = id

        # Create all action and service for the navigation system.
        self.get_logger().info(
            'Init all actions and service for navigation')

        self.create_move_to()

    def create_move_to(self):
        self.get_logger().info(
            'Init Move To primitive for the robot %d' % self.id)
        mt = MoveToPrimitive(self.id, self)

        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            mt.handler_callback)


def main(args=None):
    rclpy.init(args=args)
    id = 0

    navigation = Navigation(id)

    rclpy.spin(navigation)


if __name__ == '__main__':
    main()
