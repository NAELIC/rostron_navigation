from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from .primitive import Primitive
from rostron_interfaces.action import MoveTo
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist

class MoveToPrimitive(Primitive):
    def __init__(self, id: int, node: Node) -> None:
        super().__init__()
        self.id = id
        self.node = node  # TODO : Move this with the new utils package

    def handler_callback(self, goal_handle : ServerGoalHandle) -> None:
        self.node.get_logger().info('Executing goal...')
        pose : Pose = goal_handle.request.pose
        self.node.get_logger().info('%lf' % pose.position.x)
        print("Destination :", (pose.position.x,pose.position.y))

        robot = MoveToStrategie()    
        robot.move_to(self.id,(pose.position.x,pose.position.y))

        goal_handle.succeed()    
        result = MoveTo.Result()
        return result

import math
from math import sin, cos
import numpy as np

import rclpy
from ..path_planning.aStar_to_ros import get_path_a_star

from ..rviz_vizualisation import RvizVizualisation

from rostron_interfaces.msg import Robots, Order

class MoveToStrategie(Node):
    #poseRobots= Robots()
    def __init__(self):
        super().__init__("move_to")
        self.subscription = self.create_subscription(
            Robots,
            '/yellow/allies',
            self.listener_robots,
            1
        )
        self.subscription
        self.poseRobots = Robots()
        self.publisher = self.create_publisher(Order,'/yellow/order', 1)

    def listener_robots(self, robots):
        self.poseRobots = robots
        return robots

    """
        Utilisation de numpy !
    """
    def normeVecteur (self, x, y):
        return math.sqrt( x**2 + y**2 )

    def normaliserVecteur(self, vecteur):
        return  vecteur / np.sqrt(np.sum(vecteur**2))

    def distance (self, x, y, xa, ya):
        return math.sqrt(((xa-x)**2)+((ya-y)**2))


    """
        Regarder tf de ros....
        Numpy propose sinon quelque chose dans cette veine !
    """
    def matriceRotation(self, x ,y, theta):
        # [cos(θ) -sin(θ)]   [x]
        # [sin(θ)  cos(θ)] * [y]
        return( cos(theta)*x - sin(theta)*y , sin(theta)*x+cos(theta)*y )
    
    def order_robot(self, id, path):
        for pose in path:
            rclpy.spin_once(self)
            robotIdPose = self.poseRobots.robots[id].pose # Robot 0

            ArriveX=pose[0]
            ArriveY=pose[1]

            # 1. récupérer: position et orientation courante du robot + position d'arrivé les deux étant dans le repère du terrain
            robotX = robotIdPose.position.x
            robotY = robotIdPose.position.y
            robotO = robotIdPose.orientation.z

            # 2. calculer le vecteur allant de la position du robot à la position d'arrivée
            vecteur = (ArriveX-robotX, ArriveY-robotY)
        
            # 3. créer une matrice de rotation anti-horaire correspondant à l'orientation du robot
            # 4. appliquer cette matrice sur le vecteur
            vecteur = self.matriceRotation(vecteur[0],vecteur[1],robotO)

            robotX = self.poseRobots.robots[id].pose.position.x
            robotY = self.poseRobots.robots[id].pose.position.y
            while self.distance(robotX, robotY, ArriveX, ArriveY)>0.1:
                rclpy.spin_once(self) # get current poseRobots
                robotX = self.poseRobots.robots[id].pose.position.x
                robotY = self.poseRobots.robots[id].pose.position.y
                if self.distance(robotX, robotY, ArriveX, ArriveY)>0.3 or pose==path[-1]:
                    vecteur = (ArriveX-robotX, ArriveY-robotY)
                    vecteur = self.matriceRotation(vecteur[0],vecteur[1],robotO)
                vel_msg = Twist()
                vel_msg.linear.x= vecteur[0]
                vel_msg.linear.y= vecteur[1]
                msg = Order()
                msg.id = id
                msg.velocity = vel_msg
                self.publisher.publish(msg)
        self.stop_robot(id)

    def stop_robot(self, id):
        msg = Order()
        msg.id = id
        msg.velocity = Twist()
        self.publisher.publish(msg)
        

    def calculate_angle(self, u): # u -> vector
        v = (1,0)# x-axe
        prodscal = u[0] * v[0] + u[1] * v[1]
        NormeU = np.sqrt(u[0]**2 + u[1]**2)
        NormeV = np.sqrt(v[0]**2 + v[1]**2)
        return round(np.arccos( prodscal / (NormeU * NormeV) ) * 180 / np.pi,0)

    def optimized_path(self, path):
        new_path = [path[0],path[1]]
        vector1 = (path[1][0]-path[0][0],path[1][1]-path[0][1]) #(Xb-Xa , Yb-Ya)
        mem_angle = self.calculate_angle(vector1)
        for i in range(1,len(path)-1):
            cur_value = path[i]
            next_value = path[i+1]
            vector2 = (next_value[0]-cur_value[0],next_value[1]-cur_value[0])
            cur_angle = self.calculate_angle(vector2)
            print("----------------------------------")
            print("diff. ", abs(mem_angle - cur_angle)," | angle: ",cur_angle, " | position courante: ",cur_value, " :=: ",next_value)
            if abs(mem_angle != cur_angle) :
                print("accepted !")
                new_path.append(path[i])
                mem_angle = cur_angle
            else :
                print("rejected !")
        new_path.append(path[-1])
        return new_path

    def move_to(self, id, finalPose):
        rclpy.spin_once(self)

        path= get_path_a_star(id,finalPose)
        """
        print("Ancien path:",path)
        path = self.optimized_path(path)
        print( "Nouveau path:",path)
        """
        self.order_robot(id, path)

        rviz = RvizVizualisation()
        rviz.get_rviz_vizualisation(path)

        self.destroy_node()