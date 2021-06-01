from ..rviz_vizualisation import RvizVizualisation
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Point
from rostron_interfaces.action import MoveTo
from rostron_utils.world import World

from .primitive import Primitive
from ..path_planning.a_star import AStar


class MoveToPrimitive(Primitive):
    def __init__(self, id: int) -> None:
        super().__init__()
        self.id = id

    def handler_callback(self, goal_handle: ServerGoalHandle) -> None:
        goal : MoveTo.Goal = goal_handle.request
        World().node_.get_logger().info('[MOVETO] - Executing MoveTo (%.2lf, %.2lf, %.2lf)' % (
            goal.position.x, goal.position.y, goal.orientation))

        a = AStar(self.id, goal)
        a.run()
        RvizVizualisation().get_rviz_map(a)
        
        goal_handle.succeed()
        result = MoveTo.Result()
        return result