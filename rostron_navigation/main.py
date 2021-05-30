import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rostron_interfaces.action import MoveTo
from rostron_utils.world import World


from .primitive.move_to import MoveToPrimitive


class Navigation(Node):

    def __init__(self, id: int) -> None:
        super().__init__('navigation')
        self.id = id
        World().init(self)

        # Create all action and service for the navigation system.
        self.get_logger().info(
            'Init all actions and service for navigation')

        self.create_move_to()

        self.get_logger().info(
            'Finish all action')

    def create_move_to(self):
        self.get_logger().info(
            'Init MoveTo primitive for the robot %d' % self.id)
        self.get_logger().info(
            'Init MoveTo')

        mt = MoveToPrimitive(self.id)

        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            mt.handler_callback)
        self.get_logger().info(
            'finish init')


def main(args=None):
    rclpy.init(args=args)
    id = 0
    navigation = Navigation(id)
    # waitable = WorldWaitable(navigation)
    # navigation.add_waitable(WorldWaitable)

    # navigation.get_logger().info('test')
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
