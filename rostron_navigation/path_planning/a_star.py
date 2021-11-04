from rostron_interfaces.msg import Robot

from rostron_utils.world import World

from geometry_msgs.msg import Pose2D

import numpy as np
import math
import seaborn as sns
import matplotlib.pyplot as plt

class AStar():
    resolution = 0.1
    max_robots = 16

    def __init__(self):
        # Get width, height and add a margin
        self.width = World().field.width + 1.0
        self.length = World().field.length + 1.0
        self.xlen_ = math.ceil(self.length / self.resolution)
        self.ylen_ = math.ceil(self.width / self.resolution)

    def grid_to_coord(self, i, j):
        x = i * self.resolution - (self.length / 2.0)
        y = j * self.resolution - (self.width / 2.0)
        return ( int(x), int(y))

    def coord_to_grid(self, x, y):
        i = (x + (self.length / 2.0)) / self.resolution
        j = (y + (self.width / 2.0)) / self.resolution
        return (int(i), int(j))

    def run(self, id: int, debug : bool = False):
        # Initialize the grid
        World().node_.get_logger().info("Grid - creation")
        grid = np.zeros((self.xlen_, self.ylen_))

        World().node_.get_logger().info(f"{ self.xlen_} x {self.ylen_}")

        World().node_.get_logger().info("Grid - filling")
        for i in range(6):
            posA = self.coord_to_grid(World().allies[i].pose.position.x, World().allies[i].pose.position.y)
            posO = self.coord_to_grid(World().opponents[i].pose.position.x, World().opponents[i].pose.position.y)
            World().node_.get_logger().info("test finish 22")

            if id != i :
                grid[posA[0], posA[1]] =1
            grid[posO[0], posO[1]]=1        
            World().node_.get_logger().info("test finish 23")

            adjacent_squares=[(0,0),(0,1),(1,0),(-1,0),(0,-1),(-1,-1),(1,1),(-1,1),(1,-1),(0,-2),(0,2),
            (-2,0),(2,0),(2,-1),(2,1),(-2,1),(-2,-1),(-1,-2),(1,-2),(1,-2),(-1,2),(1,2)]
            for adjacent in adjacent_squares:
                if id != i :
                    grid[posA[0]+adjacent[0], posA[1]+adjacent[1]]=1
                grid[posO[0]+adjacent[0], posO[1]+adjacent[1]] = 1
        # for i in range(grid.shape[0]):
        #     for j in range(grid.shape[1]):
        #         pos = self.grid_to_coord(i, j)

        #         for r_id in range(16):
        #             r_ally: Robot = World().allies[r_id]

        #             if r_ally.id != id and r_ally.active == True:
        #                 r_pos = np.array((r_ally.pose.position.x, r_ally.pose.position.y))
                        
        #                 dist = np.linalg.norm(r_pos - np.array(pos))

        #                 if dist > self.resolution:
        #                     grid[i, j] = max(grid[i, j], 10.0 / (dist / self.resolution))

        #             r_opp: Robot = World().opponents[r_id]
        #             if r_opp.active == True:
        #                 r_pos = np.array((r_ally.pose.position.x, r_ally.pose.position.y))
        #                 dist = np.linalg.norm(r_pos - np.array(pos))

        #                 if dist > self.resolution:
        #                     grid[i, j] = max(grid[i, j], 10.0 / (dist / self.resolution))

                                
        if debug:
            self.print_heatmap(grid)

    def print_heatmap(self, grid):
        sns.set_theme()
        sns.heatmap(grid)
        plt.show()