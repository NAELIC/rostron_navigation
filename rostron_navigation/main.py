import rclpy
from rclpy.node import Node

from rostron_utils.world import World

from .primitive.move_to import MoveToPrimitive


class Navigation(Node):
    id = 0

    def __init__(self) -> None:
        super().__init__('navigation')
        self.declare_parameter('id', 0)
        self.id = self.get_parameter('id').get_parameter_value().integer_value

        World().init(self)

        # Create all action and service for the navigation system.
        self.get_logger().info(
            f'Init actions and service for robot {self.id}')

        self.create_move_to()

        self.get_logger().info(
            'Finish to init all action and service')

    def create_move_to(self):
        self.get_logger().info('Init MoveTo primitive')
        MoveToPrimitive(self.id, self)


def main():
    rclpy.init()
    navigation = Navigation()
    rclpy.spin(navigation)

    navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
