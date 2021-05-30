import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class RvizVizualisation(Node):
    """
    ## Goal
    With rviz, you can see trajectories of robots, the location of other robots and the map

    ## Integration 
    - In a new terminal (after launching ros), launch rviz with the command : rviz2
    - On Rviz2 : Add a new display (Ctrl+N) and choose Path then Confirm
    - In the new section Path : Rename the topic name in  'yellow/path'
    """
    def __init__(self):
        super().__init__('rviz_vizualisation')
        self.publisher_ = self.create_publisher(Path, '/yellow/path', 10)

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

if __name__ == '__main__':
    #main()
    pass