import math
import numpy as np

from rosidl_parser.definition import Array
from .square_map import SquareMap

from .aStar import a_star

#c'est moche mais pour le moment ça fait le taff
from .world import World

import rclpy
from rclpy.node import Node
from rostron_interfaces.msg import Robots
from std_msgs.msg import String


class PositionRobots(Node):

    def __init__(self):
        super().__init__('position_robots')
        self.subscription = self.create_subscription(
            Robots,
            '/yellow/allies',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.position_robots = Robots

    def listener_callback(self, robots):
        self.position_robots=robots


"""
    E.S : Une classe serait mieux et permettrait de créer une grille de x * y.
    Proposer un constructeur peut-être en fonction de la résolution.
"""
# Create a map of SSL
#SquareMap = [[0 for x in range(98)] for y in range(68)]
map = SquareMap(98,68)

# transposes a real position to a position on the map (SquareMap type)
def pose_to_square(x, y) -> tuple:
    return (math.floor(10*(x + 4.9)),math.floor(10 * ( y + 3.4)))

def square_to_pose(x, y):
    poseX= -4.9+0.1*x+0.05
    poseY= -3.4+0.1*y+0.05
    return (round(poseX, 1),round(poseY, 1))

def distance (x, y, xa, ya):
        """ Calculate the distance between two points"""
        return math.sqrt(((xa-x)**2)+((ya-y)**2))

# if robot is in a square : square = -1
def RobotisInSquare(id, square, robots, squareMap) -> None:
    pose = square_to_pose(square[0],square[1])
    #ppose = np.array((pose[0],pose[1],0))

    radius=0.085
    distSquare= 0.06+radius
    
    for i in range(12):
        #positionRobot = np.array(robots[i].pose.position.x,robots[i].pose.position.y,0)#position robot i
        #distRobot = np.linalg.norm(positionRobot - ppose)
        distRobot= distance(pose[0],pose[1],robots[i].pose.position.x,robots[i].pose.position.y)
        if distRobot<distSquare and i!=id : 
            squareMap.set_xy_map(square,-1) #-1 for obstacles


def get_path_a_star(id,arrival_pose):
    squareMap = SquareMap(98,68)
    robots=PositionRobots()
    rclpy.spin_once(robots)
    robot_position_square = pose_to_square(robots.position_robots.robots[id].pose.position.x , robots.position_robots.robots[id].pose.position.y)
    
    #All obstacles
    """
    for i in range(len(squareMap.map)):
        for j in range(len(squareMap.map[i])):
            pose = square_to_pose(i,j)
            #ppose = np.array((pose[0],pose[1],0))

            radius=0.085
            distSquare= 0.06+radius
            for i in range(12):
                #positionRobot = np.array(robots[i].pose.position.x,robots[i].pose.position.y,0)#position robot i
                #distRobot = np.linalg.norm(positionRobot - ppose)
                distRobot= distance(pose[0],pose[1] , robots.position_robots.robots[id].pose.position.x , robots.position_robots.robots[id].pose.position.y)
                print(distRobot)
                if distRobot<distSquare and i!=id : 
                    print("JE SUIS UN OBSTACLE")
                    squareMap.set_xy_map((i,j),-1) #-1 for obstacles
    """

    for i in range(6):
        #positionRobot = np.array(robots[i].pose.position.x,robots[i].pose.position.y,0)#position robot i
        #distRobot = np.linalg.norm(positionRobot - ppose)
        x = robots.position_robots.robots[i].pose.position.x
        y = robots.position_robots.robots[i].pose.position.y 
        square = pose_to_square(x ,y)
        print("Robot ",i," : ",square,"x et y :",x,y)
        adjacent_squares=[(0,0),(0,1),(1,0),(-1,0),(0,-1),(-1,-1),(1,1),(-1,1),(1,-1),(0,-2),(0,2),(-2,0),(2,0),(2,-1),(2,1),(-2,1),(-2,-1),(-1,-2),(1,-2),(1,-2),(-1,2),(1,2)]
        for adjacent in adjacent_squares:
            if id != i :
                squareMap.set_xy_map((square[0]+adjacent[0],square[1]+adjacent[1]),-1)
    

    count=0
    for i in range(len(squareMap.map)):
        for j in range(len(squareMap.map[i])):
            if squareMap.map[i][j]==-1:
                count=count+1
    print("NB Obstacles :",count)

    squareMap.set_xy_map((robot_position_square),1)
    
    #Square Arrival
    print("square arrivée",pose_to_square(arrival_pose[0] , arrival_pose[1]))
    squareMap.set_xy_map(pose_to_square(arrival_pose[0] , arrival_pose[1]),2)

                

    # Create the path to SquareArrival
    path = a_star(squareMap.map)
    print(path)
    #Transform path with squares in path with poses
    for i in range(len(path)) : 
        path[i]= square_to_pose(path[i][0],path[i][1])
    
    print(path)
    return path
    
    
if __name__ == "__main__":
    pass