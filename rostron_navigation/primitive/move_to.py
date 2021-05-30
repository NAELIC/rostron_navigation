import math
from math import sin, cos
import rclpy
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from rostron_interfaces.action import MoveTo
from rostron_interfaces.msg import Robots, Order

from .primitive import Primitive
from ..path_planning.aStar_to_ros import AStar
from ..rviz_vizualisation import RvizVizualisation

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
        a = AStar(self.id,(pose.position.x,pose.position.y), 0.0)
        path= a.run()
        robot.move_to(self.id,path)

        RvizVizualisation().get_rviz_vizualisation(path)

        goal_handle.succeed()    
        result = MoveTo.Result()
        return result

class MoveToStrategie(Node):
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
        self.finished=False

    def listener_robots(self, robots):
        self.poseRobots = robots
        return robots

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
    
    def move_to(self, id, path):
        """Move to the last pose of the path going through all poses"""
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

            while self.distance(robotX, robotY, ArriveX, ArriveY)>0.1 :
                rclpy.spin_once(self) # get current poseRobots
                robotX = self.poseRobots.robots[id].pose.position.x
                robotY = self.poseRobots.robots[id].pose.position.y
               
                if self.distance(robotX, robotY, ArriveX, ArriveY)>0.3 or pose==path[-1]:
                    # solution pour que le robot ne s'arrête pas à chaque pose
                    # à 30cm de sa destination il garde le même vecteur et donc ne ralentit pas
                    # exception pour le dernier pose où il ralentit
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
        """Give at the robot a nullable velocity"""
        msg = Order()
        msg.id = id
        msg.velocity = Twist()
        self.publisher.publish(msg)
        print("Destination OK")
        self.finished=True