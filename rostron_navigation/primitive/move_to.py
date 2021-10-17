import math
from math import sin, cos
import numpy as np

import rclpy
from rostron_interfaces.msg import Order, Orders, Hardware
from geometry_msgs.msg import Twist
from rostron_utils.world import World

from .primitive import Primitive
from ..path_planning.path_finder import PathFinder
from ..path_planning.a_star import AStar

"""
/!\ Below this line, the code needs to be removed ! /!\ 
"""

# TODO : Remove this
class R_Math():
    def angle_between(self, v_1, v_2):
        """Returns the angle in radians between vectors 'v1' and 'v2'"""
        v1_u = v_1 / np.linalg.norm(v_1)  # unit vector
        v2_u = v_2 / np.linalg.norm(v_2)  # unit vector
        sign = 1 if np.cross(v1_u, v2_u) > 0 else -1
        return sign * np.arccos(np.dot(v1_u, v2_u))

# TODO : Use primtive abstract class
# TODO : Handle Kick and dribble ?
class MoveTo():
    """
        TODO : Improve speed and stability speed.
        TODO : If we want to go in the same point there is a path finding error
    """

    def __init__(self, id):
        # super().__init__()
        self.publisher = World().node_.create_publisher(Orders, 'orders', 1)
        self.id = id
        self.rviz = False
        self.closest_target = 0.1

    def distance(self, pos, pos2):
        return math.sqrt(((pos2.x - pos[0])**2)+((pos2.y - pos[1] )**2))

    def matrice_rotation(self, vec, orientation):
        # [cos(θ) -sin(θ)]   [x]
        # [sin(θ)  cos(θ)] * [y]
        theta = -orientation  # Chercher pourquoi c'est inversé
        rotation = np.array(((np.cos(theta), -np.sin(theta)),
                             (np.sin(theta),  np.cos(theta))))
        return rotation.dot(vec)

    def theta_to_vector(self, theta):
        """Returns a vector associated with an angle theta"""
        return (cos(theta), sin(theta))

    def closest_angle(self, theta):
        """
            Find the direction of speed thanks to the requested angle and 
            orientation of the robot
        """
        orientation = World().allies[self.id].pose.orientation.z
        angle = R_Math().angle_between(self.theta_to_vector(orientation),
                                       self.theta_to_vector(theta))
        return 1 if angle > 0 else -1

    def speed_multiplier(self, arrival_pose):
        distance = 2*self.distance(arrival_pose, World().allies[self.id].pose.position)
        distance = max(distance, 1.0)
        distance = min(distance, 6.0)
        return distance

    def move_to(self, pose: tuple) -> bool:
        # TODO : Move PathFinder and A* in the same class
        
        path_finder = PathFinder(self.id, pose)
        World().node_.get_logger().info("finish path finder")

        path = path_finder.run(AStar)
        

        for goal_pose in path:
            sign = self.closest_angle(pose[2])

            robot_pose = (World().allies[self.id].pose.position.x,
                          World().allies[self.id].pose.position.y)

            robot_orientation = World().allies[self.id].pose.orientation.z
            
            vecteur = (goal_pose[0] - robot_pose[0],
                       goal_pose[1] - robot_pose[1])
            vecteur = self.matrice_rotation(vecteur, robot_orientation)
            World().node_.get_logger().info("test")
            
            control = Order()
            control.id = self.id
            
            vel_msg = Twist()
            vel_msg.linear.x = vecteur[0] * self.speed_multiplier(path[-1])
            vel_msg.linear.y = vecteur[1] * self.speed_multiplier(path[-1])
            control.velocity = vel_msg

            if abs(robot_orientation - pose[2]) > 0.1:
                vel_msg.angular.z = sign*0.8
        msg = Orders()
        msg.orders.append(control)
        self.publisher.publish(msg)

        return False
