import math
import numpy as np
import time

from rostron_utils.world import World

from geometry_msgs.msg import Point

class PathFinder():

    def __init__(self, id: int, goal : tuple) -> None:
        self.resolution = 0.1
        self.margin = 4
        self.width = World().field.width
        self.length = World().field.length

        self.x_ = math.ceil(self.width / self.resolution) + self.margin
        self.y_ = math.ceil(self.length / self.resolution) + self.margin
        
        self.id_ = id
        self.goal_ = goal
    
    def create_grid(self):
        """ Create a 2D array of the map with obstacles (robot)"""
        grid = np.zeros((self.y_, self.x_))
        
        # Obstacles
        # TODO : Error, allies is not to 0-6 but to 0-15 !!
        for i in range(6):
            World().node_.get_logger().info("test finish 21")

            posA = self.pose_to_grid(World().allies[i].pose.position)
            posO = self.pose_to_grid(World().opponents[i].pose.position)
            World().node_.get_logger().info("test finish 22")

            if self.id_ != i :
                grid[posA[0]][posA[1]]=1
            grid[posO[0]][posO[1]]=1        

            adjacent_squares=[(0,0),(0,1),(1,0),(-1,0),(0,-1),(-1,-1),(1,1),(-1,1),(1,-1),(0,-2),(0,2),
            (-2,0),(2,0),(2,-1),(2,1),(-2,1),(-2,-1),(-1,-2),(1,-2),(1,-2),(-1,2),(1,2)]
            for adjacent in adjacent_squares:
                if self.id_ != i :
                    grid[posA[0]+adjacent[0]][posA[1]+adjacent[1]]=1
                grid[posO[0]+adjacent[0]][posO[1]+adjacent[1]]=1
        return grid

    def create_1d_grid(self):
        """ Return the grid in a 1D array"""
        return (self.create_grid()).flatten()

    def pose_to_grid(self, position) -> tuple:
        """Transposes a real position to a position on the grid"""
        return ( math.floor(((position.x + self.length/2)/self.resolution)+self.margin/2), 
                math.floor(-((position.y - self.width/2)/self.resolution)+self.margin/2))

    def grid_to_pose(self, grid) -> tuple:
        """Transpose a position in the grid into a real position"""
        return ( -self.length/2 + self.resolution*grid[0]+(self.resolution/2) -(self.margin/2)*self.resolution,   
                self.width/2 - self.resolution*grid[1]-(self.resolution/2) + (self.margin/2)*self.resolution)     

    def run(self, path_finding_class):
        """ The path_finding class in parameter must have
        * 3 arguments : grid, start and goal
        * a run method which return a path"""
        start_time = time.time()
        grid = self.create_grid()
        start = self.pose_to_grid(World().allies[self.id_].pose.position)

        point = Point()
        point.x = self.goal_[0]
        point.y = self.goal_[1]

        goal = self.pose_to_grid(point)


        path_finding = path_finding_class(grid,start, goal)
        path = path_finding.run()
        for i in range(len(path)):
            path[i] = self.grid_to_pose(path[i])
        World().node_.get_logger().info('[PATH] - Generation Time : %.2lf ms' % (
            round((time.time()-start_time)*1000,2)))
        return path