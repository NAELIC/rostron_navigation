import rclpy
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import PoseStamped, Pose, Point
from rclpy.node import Node
from .path_planning.path_finder import PathFinder

from rostron_utils.world import World

class RvizVizualisation(Node):
    """
    Rviz is a viewer, thanks to this, you can see trajectories of robots, the location of other robots and the map
    """
    def __init__(self) -> None:
        super().__init__('rviz_vizualisation')
        self.publisher_path = World().node_.create_publisher(Path, '/yellow/path', 10)
        self.publisher_map= World().node_.create_publisher(OccupancyGrid, '/yellow/map', 10)
        self.publisher_grid_cell= World().node_.create_publisher(GridCells, '/yellow/grid_cell', 10)


    def get_rviz_path(self, path):
        """Show the path on rviz"""
        goal_msg = Path()
        goal_msg.poses = []
        goal_msg.header.frame_id = 'map'
        for pose in path : 
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = -pose[1]
            msg.pose.position.y = pose[0]
            goal_msg.poses.append(msg)
        self.publisher_path.publish(goal_msg)
        rclpy.spin_once(self)
        self.destroy_node()
        
    def get_rviz_map(self, grid : PathFinder ):
        """Show the map and robots on the rviz map"""
        origin = Pose()
        origin.position.x = -grid.width/2 -(grid.margin/2)*grid.resolution
        origin.position.y = -grid.length/2 -(grid.margin/2)*grid.resolution

        metaData = MapMetaData()
        metaData.height = int(grid.margin+(grid.length/grid.resolution))
        metaData.resolution = grid.resolution
        metaData.width = int(grid.margin+(grid.width/grid.resolution))
        metaData.origin = origin

        goal_msg = OccupancyGrid()
        goal_msg.header.frame_id = 'map'
        goal_msg.info = metaData
        
        data = grid.create_1d_grid() 
        data = [ int(x) for x in data ]
        goal_msg.data= data
        self.publisher_map.publish(goal_msg)
        rclpy.spin_once(self)
        self.destroy_node()
    
    def get_rviz_grid(self, grid : PathFinder):
        """Show the grid cell"""
        cells = []
        for i in range(grid.x_):
            for j in range(grid.y_):
                point = Point()
                pose = grid.grid_to_pose((i,j))
                point.x = pose[0]+ grid.resolution/2
                point.y = pose[1] + grid.resolution/2
                cells.append(point)
        
        goal_msg = GridCells()
        goal_msg.header.frame_id = 'map'
        goal_msg.cell_height = grid.resolution
        goal_msg.cell_width = grid.resolution
        goal_msg.cells = cells        
        self.publisher_grid_cell.publish(goal_msg)
        rclpy.spin_once(self)


if __name__ == '__main__':
    #main()
    pass