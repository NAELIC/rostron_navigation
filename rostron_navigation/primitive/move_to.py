from ..rviz_vizualisation import RvizVizualisation
from rclpy.action.server import ServerGoalHandle
from rostron_interfaces.action import MoveTo
from rostron_utils.world import World
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from math import sin, cos
import numpy as np
import rclpy
from rostron_interfaces.msg import Order, Robots

from .primitive import Primitive
from ..path_planning.path_finder import PathFinder
from ..path_planning.a_star import AStar

class MoveToPrimitive(Primitive):
    def __init__(self, id: int) -> None:
        super().__init__()
        self.id = id

    def handler_callback(self, goal_handle: ServerGoalHandle) -> None:
        goal : MoveTo.Goal = goal_handle.request
        World().node_.get_logger().info('[MOVETO] - Executing MoveTo (%.2lf, %.2lf, %.2lf)' % (
            goal.position.x, goal.position.y, goal.orientation))

        path_finder = PathFinder(self.id, goal)
        path = path_finder.run(AStar)

        RvizVizualisation().get_rviz_path(path)
        RvizVizualisation().get_rviz_map(path_finder) 
        #RvizVizualisation().get_rviz_grid(path_finder)

        robot = MoveToStrategie(self.id)
        robot.move_to(path)

        goal_handle.succeed()
        result = MoveTo.Result()
        return result


class MoveToStrategie(Node):
    def __init__(self, id):
        super().__init__("move_to")
        self.publisher = self.create_publisher(Order,'/yellow/order', 1)
        self.id = id
        self.robot_position=(World().allies[self.id].pose.position.x,World().allies[self.id].pose.position.y)
        self.robot_orientation = World().allies[self.id].pose.orientation.z

    def update_pose_robot(self):
        """Update the values ​​of positional attributes"""
        self.robot_position=(World().allies[self.id].pose.position.x, World().allies[self.id].pose.position.y)
        self.robot_orientation = World().allies[self.id].pose.orientation.z

    def distance (self, pos, pos2):
        return math.sqrt(((pos2[0]-pos[0])**2)+((pos2[1]-pos[1])**2))

    def matriceRotation(self, vec, theta):
        # [cos(θ) -sin(θ)]   [x]
        # [sin(θ)  cos(θ)] * [y]
        return( cos(theta)*vec[0] - sin(theta)*vec[1] , sin(theta)*vec[0]+cos(theta)*vec[1])

    def unit_vector(self,vector):
        """Returns the unit vector of the vector."""
        return vector / np.linalg.norm(vector)

    def angle_between(self,v1, v2):
        """Returns the angle in radians between vectors 'v1' and 'v2'"""
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        sign = 1 if np.cross(v1_u, v2_u) > 0 else -1 
        return sign * np.arccos(np.dot(v1_u, v2_u)) 

    def pose_to_direction(self,arrival_pose,vector):
        self.update_pose_robot()
        dist_arrival = self.distance(self.robot_position, arrival_pose)
        angle = self.angle_between(vector,(1,0))
        if dist_arrival>0.5:
            dist_arrival=0.5
        return (self.robot_position[0]+dist_arrival*cos(angle),self.robot_position[1]+dist_arrival*sin(angle))
    
    def speed_multiplier(self, arrival_pose):
        distance = 3*self.distance(arrival_pose, self.robot_position)
        if distance < 1 :
            distance=1
        if distance > 5.5 :
            distance=5.5
        return distance 

    def move_to(self, path):
        """Move to the last pose of the path going through all poses"""
        for goal_pose in path:
            rclpy.spin_once(self)
            self.update_pose_robot()
            vecteur = (goal_pose[0]-self.robot_position[0], goal_pose[1]-self.robot_position[1])
            vecteur = self.matriceRotation(vecteur,self.robot_orientation)

            while self.distance(self.robot_position, goal_pose)>0.1 :
                self.update_pose_robot()
                vecteur = (goal_pose[0]-self.robot_position[0], goal_pose[1]-self.robot_position[1])
                vecteur = self.matriceRotation(vecteur,self.robot_orientation)
                vel_msg = Twist()
                vel_msg.linear.x= self.speed_multiplier(path[-1])*vecteur[0]
                vel_msg.linear.y= self.speed_multiplier(path[-1])*vecteur[1]
                msg = Order()
                msg.id = self.id
                msg.velocity = vel_msg
                self.publisher.publish(msg)
                rclpy.spin_once(self)
        self.stop_robot()
    
    def stop_robot(self):
        """Give at the robot a nullable velocity"""
        msg = Order()
        msg.id = self.id
        msg.velocity = Twist()
        self.publisher.publish(msg)
        rclpy.spin_once(self)
        World().node_.get_logger().info('[MOVETO] - Goal Achieved !')
