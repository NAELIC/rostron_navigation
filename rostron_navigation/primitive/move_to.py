import math
from math import sin, cos
import time
import numpy as np

import rclpy
from rostron_interfaces.msg import Order, Orders, Hardware
from geometry_msgs.msg import Twist
from rostron_utils.world import World

from .primitive import Primitive
from ..path_planning.path_finder import PathFinder
from ..path_planning.a_star import AStar

from rostron_ia_ms.main import IANode

import matplotlib.pyplot as plt 
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


        self.robot_position = (World().allies[self.id].pose.position.x,
                            World().allies[self.id].pose.position.y)
        self.node = IANode()
        self.current_location=(self.robot_position, time.time())
        self.speed_over_time=[[0.0,0.0]] #  [[speed,time],...]

    def distance(self, pos, pos2):
        return math.sqrt(((pos2[0] - pos[0])**2)+((pos2[1] - pos[1] )**2))

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
        distance = 2*self.distance(arrival_pose, 
                    (World().allies[self.id].pose.position.x,World().allies[self.id].pose.position.y))
        distance = max(distance, 1.0)
        distance = min(distance, 6.0)
        return distance

    def speed_acceleration_vector(self, vector):
        """Return the speed of acceleration of a vector (x,y)
           Example : v (1,1) return sqrt(2)
        """
        return math.hypot(vector[0],vector[1])

    def get_acceleration_max(self, robot_pose, arrival_pose):
        """Update the acceleration max according to distance to goal"""
        distance = self.distance(robot_pose, arrival_pose)
        return 1.0 if distance>1 else distance/2
  

    def max_acceleration_vector(self, vector, max_acceleration):
        """Return the biggest vector where vector speed close to accel_max"""
        initial_vector = vector
        accel_multiplier = 1.0
        while self.speed_acceleration_vector(vector) < max_acceleration:
            vector = (initial_vector[0]*accel_multiplier,initial_vector[1]*accel_multiplier)
            accel_multiplier= accel_multiplier +0.1
        return vector

    def stop_robot(self, goal_orientation):
        """Give at the robot a nullable velocity when there is the right orientation"""
        
        """
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
        """
        control = Order()
        control.id = self.id
        control.velocity = Twist()

        msg = Orders()
        msg.orders.append(control)
        self.publisher.publish(msg)
        # rclpy.spin_once(self.node)
        World().node_.get_logger().info('[MOVETO] - Goal Achieved !')

    def move_to(self, pose: tuple) -> bool:
        # TODO : Move PathFinder and A* in the same class
        
        path_finder = PathFinder(self.id, pose)

        path = path_finder.run(AStar)
        
        for goal_pose in path:

            self.robot_position = (World().allies[self.id].pose.position.x,
                          World().allies[self.id].pose.position.y)
            while self.distance(self.robot_position, goal_pose) > 0.1:

                sign = self.closest_angle(pose[2])
                
                self.robot_position = (World().allies[self.id].pose.position.x,
                            World().allies[self.id].pose.position.y)
                

                robot_orientation = World().allies[self.id].pose.orientation.z
                
                vector = (goal_pose[0] - self.robot_position[0],
                        goal_pose[1] - self.robot_position[1])
                vector = self.matrice_rotation(vector, robot_orientation)
                
                control = Order()
                control.id = self.id
                
                vel_msg = Twist()

                max_acceleration = self.get_acceleration_max(self.robot_position, path[-1])
                vector = self.max_acceleration_vector(vector, max_acceleration)
                vel_msg.linear.x = vector[0]
                vel_msg.linear.y = vector[1]


                control.velocity = vel_msg
                """
                if abs(robot_orientation - pose[2]) > 0.1:
                    vel_msg.angular.z = sign*0.8
                """
                msg = Orders()
                msg.orders.append(control)
                self.publisher.publish(msg)

                rclpy.spin_once(self.node) # How to replace ?
        self.stop_robot(0.0)

        return False

    def speed_checker(self):
        last_location=self.current_location
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
        