"""
Même chose que les précédents.
Cette fonction n'a pas sa place ici, mais dans rostron_ia_ms (dans un premier temps).
Néanmoins on peut faire des actions Kick, ChipKick.

J'ai pas regardé en détail.
Mais en gros évite les globals, utilise numpy
Les recopies de fonctions...
"""
from rostron_ia_ms.utils.world import World
import math
from math import sin, cos, pi
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from rostron_interfaces.msg import Robots, Ball
from rostron_interfaces.msg import Order, Hardware

class MoveTo(Node):
    poseRobots= Robots()
    
    def __init__(self):
        super().__init__("striker")
    
        self.subscription = self.create_subscription(
            Robots,
            '/yellow/allies',
            self.listener_robots,
            1
        )
        self.subscription
        
        self.publisher = self.create_publisher(Order,'/yellow/order', 1)

    def listener_robots(self, robots):
        """ Get current poses of robots"""
        global poseRobots
        poseRobots = robots
        return robots

    def normeVecteur (self, vecteur):
        """ Calculate the norm of a vector """
        return math.sqrt( vecteur[0]**2 + vecteur[1]**2 )

    def normaliserVecteur(self, vecteur):
        return  vecteur / np.sqrt(np.sum(vecteur**2))

    def distance (self, x, y, xa, ya):
        """ Calculate the distance between two points"""
        return math.sqrt(((xa-x)**2)+((ya-y)**2))

    def matriceRotation(self, x ,y, theta):
        # [cos(θ) -sin(θ)]   [x]
        # [sin(θ)  cos(θ)] * [y]
        return( cos(theta)*x - sin(theta)*y , sin(theta)*x+cos(theta)*y )

    def order_robot(self, ArriveX, ArriveY, ArriveO):

        global poseRobots
        robotIdPose = poseRobots.robots[0].pose # Robot 0

        # 1. récupérer: position et orientation courante du robot + position d'arrivé les deux étant dans le repère du terrain
        robotX = robotIdPose.position.x
        robotY = robotIdPose.position.y
        robotO = robotIdPose.orientation.z

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

        if self.distance(robotX,robotY,ArriveX, ArriveY) < 0.05 : 
            vel_msg.linear.x= 0.0
            vel_msg.linear.y= 0.0 
            vel_msg.angular.z = 1.0

        
        if abs(robotO-ArriveO)<0.1 :
            vel_msg.angular.z = 0.0

        msg = Order()
        msg.id = 0
        msg.velocity = vel_msg
        msg.hardware = hardware_msg
        
        self.publisher.publish(msg)

    
    def go_ahead(self, x, y):
        global poseRobots
        robotIdPose = poseRobots.robots[0].pose
        robotX= robotIdPose.position.x
        robotY = robotIdPose.position.y

        vel_msg = Twist()
        vel_msg.linear.x= 0.5

        if self.distance(robotX,robotY,x, y) < 0.05 : 
            vel_msg.linear.x= 0.0

        hardware_msg = Hardware()
        msg = Order()
        msg.id = 0
        msg.velocity = vel_msg
        msg.hardware = hardware_msg
        
        self.publisher.publish(msg)
    
    def kick(self):
        """ Kick the ball in the current pose"""

        hardware_msg = Hardware()
        hardware_msg.spin_power=0.0
        hardware_msg.kick_type=1
        hardware_msg.kick_power=0.5

        msg = Order()
        msg.id = 0
        msg.hardware = hardware_msg
        
        self.publisher.publish(msg)


def first_pose(ArriveX,ArriveY,theta):
    """ Returns a pose close to the point and aligned with the angle theta"""
    return (ArriveX+0.3*cos(theta-math.pi), ArriveY+0.3*sin(theta-math.pi))

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    
    sign = 1 if np.cross(v1_u, v2_u) > 0 else -1 
    return sign * np.arccos(np.dot(v1_u, v2_u))

def main(args=None):
    rclpy.init(args=args)

    move_to = MoveTo()

    rclpy.spin_once(move_to)
    global poseRobots
    robotX = poseRobots.robots[0].pose.position.x
    robotY = poseRobots.robots[0].pose.position.y
    robotO = poseRobots.robots[0].pose.orientation.z
    
    target=(-4.5,0.0)
    goal= (0.5,2.0)

    vecteurAngle= (target[0]-goal[0], target[1]-goal[1])
    vecteurRobot= (1,0) # Vecteur vers la droite (angle=0.0)
    ArriveO = angle_between(vecteurRobot, vecteurAngle)
    firstPose = first_pose(goal[0],goal[1], ArriveO)

    print("Destination :",goal[0], goal[1]," Angle: ",ArriveO)

    while move_to.distance(robotX, robotY, firstPose[0], firstPose[1])>0.05 or abs(robotO-ArriveO)>0.05 :
        rclpy.spin_once(move_to) # get current poseRobots
        robotX = poseRobots.robots[0].pose.position.x
        robotY = poseRobots.robots[0].pose.position.y
        robotO = poseRobots.robots[0].pose.orientation.z
        move_to.order_robot(firstPose[0], firstPose[1], ArriveO)
    while move_to.distance(robotX, robotY, goal[0], goal[1])>0.05 :
        rclpy.spin_once(move_to) # get current poseRobots
        robotX = poseRobots.robots[0].pose.position.x
        robotY = poseRobots.robots[0].pose.position.y
        robotO = poseRobots.robots[0].pose.orientation.z
        move_to.go_ahead(goal[0], goal[1])
    print("Destination OK")
    move_to.kick()
    move_to.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    pass
    #main()