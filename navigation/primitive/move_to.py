from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from .primitive import Primitive
from rostron_interfaces.action import MoveTo
from rclpy.node import Node

from geometry_msgs.msg import Pose

class MoveToPrimitive(Primitive):
    def __init__(self, id: int, node: Node) -> None:
        super().__init__()
        self.id = id
        self.node = node  # TODO : Move this with the new utils package

    def handler_callback(self, goal_handle : ServerGoalHandle) -> None:
        self.node.get_logger().info('Executing goal...')
        pose : Pose = goal_handle.request.pose
        self.node.get_logger().info('%lf' % pose.position.x)
        
        goal_handle.succeed()
        result = MoveTo.Result()
        return result
