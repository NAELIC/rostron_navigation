import math
import numpy as np

from rostron_interfaces.action import MoveTo
from rostron_utils.world import World

# TODO(Etienne) Chercher un nom pour la classe mère
class AStar():

    def __init__(self, id: int, goal : MoveTo.Goal) -> None:
        self.resolution = 0.1
        self.margin = 3*2
        self.width = World().field.width
        self.length = World().field.length

        self.x_ = math.ceil(self.width / self.resolution) + self.margin
        self.y_ = math.ceil(self.length / self.resolution) + self.margin
        
        self.id_ = id
        self.goal_ = goal
    
    def create_grid(self):
        """ Create a grid of the map with obstacles (robot)"""
        grid = np.zeros((self.y_, self.x_))
        
        # Obstacles
        for i in range(6):
            posA = self.pose_to_grid(World().allies[i].pose.position)
            posO = self.pose_to_grid(World().opponents[i].pose.position)
            if self.id_ != i :
                grid[posA[0]][posA[1]]=1
            grid[posO[0]][posO[1]]=1

            print((World().allies[i].pose.position.x, World().allies[i].pose.position.y) ," -> ",posA," -> ", self.grid_to_pose(posA) )
            print((World().opponents[i].pose.position.x, World().opponents[i].pose.position.y) ," -> ",posO," -> ", self.grid_to_pose(posO) )
        return grid

    def create_1d_grid(self):
        """ Return the grid in a 1D array"""
        return (self.create_grid()).flatten()

    def pose_to_grid(self,position) -> tuple:
        """Transposes a real position to a position on the grid"""
        return ( math.floor(((position.x + self.length/2)/self.resolution)+self.margin/2), 
                math.floor(((position.y + self.width/2)/self.resolution)+self.margin/2))

    def grid_to_pose(self, grid) -> tuple:
        """Transpose a position in the grid into a real position"""
        return ( -self.length/2 + self.resolution*grid[0]+(self.resolution/2) -(self.margin/2)*self.resolution, 
                -self.width/2 + self.resolution*grid[1]+(self.resolution/2) -(self.margin/2)*self.resolution)


    def run(self):
        self.create_grid()