import math
from math import sin, cos
import matplotlib.pyplot as plt 
import numpy as np
import time

import rclpy
from rostron_interfaces.msg import Order, Hardware
from geometry_msgs.msg import Twist
from rostron_interfaces.action import MoveTo
from rclpy.node import Node
from rostron_utils.world import World

from .primitive import Primitive
from ..path_planning.path_finder import PathFinder
from ..path_planning.a_star import AStar
from ..rviz_vizualisation import RvizVizualisation

"""
/!\ Below this line, the code needs to be removed ! /!\ 
"""


class R_Math():
    def angle_between(self, v_1, v_2):
        """Returns the angle in radians between vectors 'v1' and 'v2'"""
        v1_u = v_1 / np.linalg.norm(v_1)  # unit vector
        v2_u = v_2 / np.linalg.norm(v_2)  # unit vector
        sign = 1 if np.cross(v1_u, v2_u) > 0 else -1
        return sign * np.arccos(np.dot(v1_u, v2_u))


class MoveToStrategie(Node):
    """
    A améliorer :
    * Améliorer la gestion de la vitesse en utilisant la rampe de vitesse (angulaire et linéaire)
    * Gérer le cas lorsqu'on veut aller au même point (path finding erreur)
    * Modifier appel MoveTo de 60ms à 16ms
    """

    def __init__(self, id):
        super().__init__("move_to")
        self.publisher = self.create_publisher(Order, 'order', 1)
        self.id = id
        self.robot_position = (World().allies[self.id].pose.position.x,
                               World().allies[self.id].pose.position.y)
        self.robot_orientation = World().allies[self.id].pose.orientation.z
        self.loop_period = 0.06  # 60ms
        self.rviz = False
        self.current_location=(self.robot_position, time.time())
        self.speed_over_time=[[0.0,0.0]] #  [[speed,time],...]
        World().init(self)

    def update_pose_robot(self):
        """Update the values ​​of positional attributes"""
        self.robot_position = (World().allies[self.id].pose.position.x,
                               World().allies[self.id].pose.position.y)
        self.robot_orientation = World().allies[self.id].pose.orientation.z

    def distance(self, pos, pos2):
        return math.sqrt(((pos2[0]-pos[0])**2)+((pos2[1]-pos[1])**2))

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
        """Find the direction of speed thanks to the requested angle and orientation of the robot"""
        self.update_pose_robot()
        angle = R_Math().angle_between(self.theta_to_vector(self.robot_orientation),
                                       self.theta_to_vector(theta))
        return 1 if angle > 0 else -1

    def speed_multiplier(self, arrival_pose):
        distance = 2*self.distance(arrival_pose, self.robot_position)
        distance = max(distance, 1.0)
        distance = min(distance, 6.0)
        return distance

    def move_by(self, path, orientation_goal, dribble=False):
        """Move to the last pose of the path going through all poses"""
        start_time = time.time()
        for goal_pose in path:
            rclpy.spin_once(self)
            self.update_pose_robot()
            sign = self.closest_angle(orientation_goal)
            while self.distance(self.robot_position, goal_pose) > 0.06:
                if time.time()-start_time > self.loop_period:
                    self.move_to(path[-1], orientation_goal, False, dribble)
                    return
                self.update_pose_robot()
                vecteur = (goal_pose[0]-self.robot_position[0],
                           goal_pose[1]-self.robot_position[1])
                vecteur = self.matrice_rotation(
                    vecteur, self.robot_orientation)
                vel_msg = Twist()
                vel_msg.linear.x = vecteur[0] * self.speed_multiplier(path[-1])
                vel_msg.linear.y = vecteur[1] * self.speed_multiplier(path[-1])

                if abs(self.robot_orientation-orientation_goal) > 0.1:
                    vel_msg.angular.z = sign*0.8
                msg = Order()
                msg.id = self.id
                msg.velocity = vel_msg
                ball_position = (World().ball.position.x,
                                 World().ball.position.y)
                if dribble and self.distance(self.robot_position, ball_position) < 0.3:
                    hardware_msg = Hardware()
                    hardware_msg.spin_power = 0.9
                    hardware_msg.kick_type = 0
                    hardware_msg.kick_power = 0.9
                    msg.hardware = hardware_msg

                self.publisher.publish(msg)
                rclpy.spin_once(self)
        self.stop_robot(orientation_goal)

    def move_and_see(self, path):
        """Move to the last pose watching the ball"""
        start_time = time.time()
        for goal_pose in path:
            self.update_pose_robot()
            rclpy.spin_once(self)
            ball_position = (World().ball.position.x, World().ball.position.y)
            vector_angle = (ball_position[0]-goal_pose[0],
                            ball_position[1]-goal_pose[1])
            vector_axis = (1, 0)  # right vector
            orientation_goal = R_Math().angle_between(vector_axis, vector_angle)
            sign = self.closest_angle(orientation_goal)

            while self.distance(self.robot_position, goal_pose) > 0.06:
                if time.time()-start_time > self.loop_period:
                    self.move_to(path[-1], orientation_goal, True)
                    return
                self.update_pose_robot()
                vecteur = (goal_pose[0]-self.robot_position[0],
                           goal_pose[1]-self.robot_position[1])
                vecteur = self.matriceRotation(vecteur, self.robot_orientation)
                vel_msg = Twist()
                vel_msg.linear.x = vecteur[0] * self.speed_multiplier(path[-1])
                vel_msg.linear.y = vecteur[1] * self.speed_multiplier(path[-1])
                if abs(self.robot_orientation-orientation_goal) > 0.15:
                    vel_msg.angular.z = sign*1.0
                msg = Order()
                msg.id = self.id
                msg.velocity = vel_msg
                self.publisher.publish(msg)
                rclpy.spin_once(self)
        self.stop_robot(orientation_goal)

    def move_to(self, pose: tuple, orientation_goal: float,
                move_and_see=False, move_and_dribble=False):
        goal = MoveTo.Goal()
        goal.position.x = pose[0]
        goal.position.y = pose[1]
        goal.orientation = orientation_goal

        path_finder = PathFinder(self.id, goal)
        path = path_finder.run(AStar)

        self.speed_checker()

        if self.rviz:
            RvizVizualisation().get_rviz_path(path)
            RvizVizualisation().get_rviz_map(path_finder)

        if move_and_see:
            self.move_and_see(path)
        else:
            if move_and_dribble:
                self.move_by(path, orientation_goal, True)
            else:
                self.move_by(path, orientation_goal)

    def stop_robot(self, goal_orientation):
        """Give at the robot a nullable velocity when there is the right orientation"""
        sign = self.closest_angle(goal_orientation)

        # if orientation not finished
        while abs(self.robot_orientation-goal_orientation) > 0.1:
            self.update_pose_robot()
            vel_msg = Twist()
            vel_msg.angular.z = sign*1.0
            msg = Order()
            msg.id = self.id
            msg.velocity = vel_msg
            self.publisher.publish(msg)
            rclpy.spin_once(self)

        msg = Order()
        msg.id = self.id
        msg.velocity = Twist()
        self.publisher.publish(msg)
        rclpy.spin_once(self)
        World().node_.get_logger().info('[MOVETO] - Goal Achieved !')

    def kick(self, kick_type):
        """ Kick the ball in the current pose"""
        hardware_msg = Hardware()
        hardware_msg.spin_power = 0.9
        hardware_msg.kick_type = kick_type
        hardware_msg.kick_power = 0.9

        msg = Order()
        msg.id = self.id
        msg.hardware = hardware_msg
        self.publisher.publish(msg)
        rclpy.spin_once(self)

    def speed_checker(self):
        last_location=self.current_location
        self.update_pose_robot()
        self.current_location=(self.robot_position, time.time())
        distance = self.distance(last_location[0],self.current_location[0])
        tim = self.current_location[1]- last_location[1]
        speed = distance/tim
        self.speed_over_time.append([speed,self.speed_over_time[-1][1]+tim])

    def plot_speed_over_time(self):
        speed = [i[0] for i in self.speed_over_time]
        time  = [i[1] for i in self.speed_over_time]
        plt.plot(time, speed, color='blue')
        plt.xlabel('Time (in sec)')
        plt.ylabel('Speed (in m/s)')

        average_speed= round(sum(speed)/len(speed),1)
        total_time= round(time[-1],1)
        distance = round(sum(speed)/len(speed)*time[-1],1)
        title= "Robot %i Path | Distance: %.1lfm | Time: %.1lfs | Average Speed: %.1lfm/s" % (self.id, distance, total_time, average_speed)
        plt.title(title)

        plt.show()

