from numpy.core.fromnumeric import transpose
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, Pose
from .path_planning.a_star import AStar

import numpy as np
import math

class RvizVizualisation(Node):
    """
    ## Goal
    With rviz, you can see trajectories of robots, the location of other robots and the map

    ## Integration 
    - In a new terminal (after launching ros), launch rviz with the command : rviz2
    - On Rviz2 : Add a new display (Ctrl+N) and choose Path then Confirm
    - In the new section Path : Rename the topic name in  'yellow/path'
    - To see the map, do the same thing with the Map and the topic name 'yellow/map'
    """
    def __init__(self):
        super().__init__('rviz_vizualisation')
        self.publisher_ = self.create_publisher(Path, '/yellow/path', 10)
        self.publisher_map= self.create_publisher(OccupancyGrid, '/yellow/map', 10)

    def get_rviz_vizualisation(self, path):
        goal_msg = Path()
        goal_msg.poses = []
        goal_msg.header.frame_id = 'map'
        for pose in path : 
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            goal_msg.poses.append(msg)
        self.publisher_.publish(goal_msg)
        rclpy.spin_once(self)
        
    
    def get_rviz_map(self, grid : AStar):
        
        origin = Pose()
        origin.position.x = -grid.width/2
        origin.position.y = -grid.length/2

        metaData = MapMetaData()
        metaData.height = int(grid.length/grid.resolution)
        metaData.resolution = grid.resolution
        metaData.width = int(grid.width/grid.resolution)
        metaData.origin = origin

        goal_msg = OccupancyGrid()
        goal_msg.header.frame_id = 'map'
        goal_msg.info = metaData
        
        #data = grid.create_1d_grid() 
        #data = [ int(x) for x in data ]
        data= np.zeros((math.ceil(grid.length / grid.resolution), math.ceil(grid.width / grid.resolution)))
        data = data.flatten()
        data = [ int(x) for x in data ]
        goal_msg.data= data
        self.publisher_map.publish(goal_msg)
        rclpy.spin_once(self)

if __name__ == '__main__':
    #main()
    pass