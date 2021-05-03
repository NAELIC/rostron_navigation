from rostron_ia_ms.utils.world import World
import math
from math import sin, cos, pi
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from rostron_interfaces.msg import Robot, Robots
from rostron_interfaces.msg import Order, Hardware

import time

class MoveTo(Node):
    poseRobots= Robots()
    def __init__(self):
        super().__init__("move_to")
        self.subscription = self.create_subscription(
            Robots,
            '/yellow/allies',
            self.listener_robots,
            1
        )
        self.subscription
        
        self.publisher = self.create_publisher(Order,'/yellow/order', 1)

    def listener_robots(self, robots):
        global poseRobots
        poseRobots = robots
        return robots

    def normeVecteur (self, x, y):
        return math.sqrt( x**2 + y**2 )

    def normaliserVecteur(self, vecteur):
        return  vecteur / np.sqrt(np.sum(vecteur**2))

    def distance (self, x, y, xa, ya):
        return math.sqrt(((xa-x)**2)+((ya-y)**2))

    def matriceRotation(self, x ,y, theta):
        # [cos(θ) -sin(θ)]   [x]
        # [sin(θ)  cos(θ)] * [y]
        return( cos(theta)*x - sin(theta)*y , sin(theta)*x+cos(theta)*y )
    
    def quaternion_to_yaw(self, x, y, z, w):
        t1 = +2.0 * (w * z + x * y)
        t2 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t1, t2)
        return yaw_z 

    def order_robot(self, ArriveX, ArriveY):

        global poseRobots
        robotIdPose = poseRobots.robots[0].pose # Robot 0

        # 1. récupérer: position et orientation courante du robot + position d'arrivé les deux étant dans le repère du terrain
        robotX = robotIdPose.position.x
        robotY = robotIdPose.position.y
        robotO = robotIdPose.orientation
        robotO = self.quaternion_to_yaw(robotO.x,robotO.y,robotO.z, robotO.w)

        # 2. calculer le vecteur allant de la position du robot à la position d'arrivée
        vecteur = (ArriveX-robotX, ArriveY-robotY)
        
        # 3. créer une matrice de rotation anti-horaire correspondant à l'orientation du robot
        # 4. appliquer cette matrice sur le vecteur
        vecteur = self.matriceRotation(vecteur[0],vecteur[1],robotO)

        # 5. limiter la vitesse du robot en limitant la norme du vecteur, si norm(vec) > MAX_SPEED alors on normalise vec
        """
        MAX_SPEED= 5.5
        if self.normeVecteur(vecteur[0], vecteur[1]) > MAX_SPEED : 
            vecteur = self.normaliserVecteur(vecteur)
        """
        vel_msg = Twist()
        vel_msg.linear.x= vecteur[0]
        vel_msg.linear.y= vecteur[1] 
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        hardware_msg = Hardware()
        hardware_msg.spin_power=0.0
        hardware_msg.kick_type=0
        hardware_msg.kick_power=0.0

        msg = Order()
        msg.id = 0
        msg.velocity = vel_msg
        msg.hardware = hardware_msg
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    move_to = MoveTo()

    rclpy.spin_once(move_to)
    global poseRobots
    robotX = poseRobots.robots[0].pose.position.x
    robotY = poseRobots.robots[0].pose.position.y

    path = [(1.0,1.0),(1.0,2.0),(2.0,2.0),(1.5,1.5),(1.0,0.8),(0.0,0.0),(-0.1,0.0),(-0.2,0.0),(-0.3,0.0),(-0.3,0.1), (-0.3,0.2), (-0.3,0.3), (-0.4,0.3), (-0.5,0.3), (-0.6,0.3), (-0.7,0.3)]
    
    for pose in path:
        ArriveX = pose[0]
        ArriveY = pose[1]
        print("Destination :",ArriveX, ArriveY)
        while move_to.distance(robotX, robotY, ArriveX, ArriveY)>0.1:
            rclpy.spin_once(move_to) # get current poseRobots
            robotX = poseRobots.robots[0].pose.position.x
            robotY = poseRobots.robots[0].pose.position.y
            move_to.order_robot(ArriveX, ArriveY)
        print("Destination OK")


    move_to.destroy_node()
    rclpy.shutdown()
       
if __name__ == '__main__':
    main()
