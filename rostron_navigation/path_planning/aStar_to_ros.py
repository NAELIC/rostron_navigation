import math
import rclpy
from rclpy.node import Node
from rostron_interfaces.msg import Robots

from .square_map import SquareMap
from .aStar import a_star

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

class AStar(Node):
    def __init__(self, id : int, position : tuple, theta : float) -> None:
        super().__init__('a_star')
        self.id = id
        self.position = position
        self.theta = theta
    
    def pose_to_square(self,x, y) -> tuple:
        """Transposes a real position to a position on the map (SquareMap type)"""
        return (math.floor(10*(x + 4.9)),math.floor(10 * ( y + 3.4)))

    def square_to_pose(self,x, y):
        """Transposes a position on the SquareMap to a real position"""
        poseX= -4.9+0.1*x+0.05
        poseY= -3.4+0.1*y+0.05
        return (round(poseX, 1),round(poseY, 1))

    def run(self):
        """- Create a map (SquareMap type) with all informations (starting, arrival, obstacles) 
        - Send this SquareMap to aStar algorithm
        - Return the path given by aStar"""
        rclpy.spin_once(self)
        squareMap = SquareMap(68,98)
        robots=PositionRobots()
        rclpy.spin_once(robots)

        # All obstacles
        for i in range(6):
            x = robots.position_robots.robots[i].pose.position.x
            y = robots.position_robots.robots[i].pose.position.y 
            square = self.pose_to_square(x ,y)
            adjacent_squares=[(0,0),(0,1),(1,0),(-1,0),(0,-1),(-1,-1),(1,1),(-1,1),(1,-1),(0,-2),(0,2),
            (-2,0),(2,0),(2,-1),(2,1),(-2,1),(-2,-1),(-1,-2),(1,-2),(1,-2),(-1,2),(1,2)]
            for adjacent in adjacent_squares:
                if self.id != i :
                    squareMap.set_xy_map((square[0]+adjacent[0],square[1]+adjacent[1]),-1)
                    
        robot_position_square = self.pose_to_square(robots.position_robots.robots[self.id].pose.position.x, 
            robots.position_robots.robots[self.id].pose.position.y)

        squareMap.set_xy_map((robot_position_square),1)

        #Square Arrival
        squareMap.set_xy_map(self.pose_to_square(self.position[0] ,self.position[1]),2)
        
        # Create the path to SquareArrival
        path = a_star(squareMap.map)

        #Transform path with squares in path with poses
        for i in range(len(path)) : 
            path[i]= self.square_to_pose(path[i][0],path[i][1])
            
        return path
    
    
if __name__ == "__main__":
    pass