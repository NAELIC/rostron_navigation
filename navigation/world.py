from rostron_interfaces.msg import Robots, Ball, Referee
from rclpy.node import Node
from rclpy.subscription import Subscription
import rclpy

import math


class SingletonMeta(type):
    """
    The Singleton class can be implemented in different ways in Python. Some
    possible methods include: base class, decorator, metaclass. We will use the
    metaclass because it is best suited for this purpose.
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class World(metaclass=SingletonMeta):
    node_: Node = None
    gc: Referee = Referee()
    ball: Ball = Ball()
    allies = []
    opponents = []

    p_ball_: Subscription = None
    p_allies_: Subscription = None
    p_opponents_: Subscription = None
    p_gc_: Subscription = None

    on_positive_half = False

    def init(self, node: Node, yellow_: bool) -> None:
        self.node_ = node
        self.yellow_ = yellow_

        # Robots
        self.p_allies_ = node.create_subscription(
            Robots,
            'allies',
            self.update_allies,
            10)

        self.p_opponents_ = node.create_subscription(
            Robots,
            'opponents',
            self.update_opponents,
            10)

        # Ball
        self.p_ball_ = node.create_subscription(
            Ball,
            'ball',
            self.update_ball,
            10)

        # GameController
        self.p_gc_ = node.create_subscription(
            Referee,
            'gc',
            self.update_gc,
            10)

    def update_allies(self, msg: Robots):
        if self.on_positive_half:
            for id, r in enumerate(msg.robots):
                # TODO: Use the ROS way for this !
                msg.robots[id].pose.position.x = -r.pose.position.x
                msg.robots[id].pose.position.y = -r.pose.position.y
                msg.robots[id].pose.orientation.z = math.fmod(
                    r.pose.orientation.z + math.pi, 2 * math.pi)
        
        self.allies = msg.robots

    def update_opponents(self, msg: Robots):
        if self.on_positive_half:
            for id, r in enumerate(msg.robots):
                msg.robots[id].pose.position.x = -r.pose.position.x
                msg.robots[id].pose.position.y = -r.pose.position.y
                msg.robots[id].pose.orientation.z = math.fmod(
                    r.pose.orientation.z + math.pi, 2 * math.pi)
    
        self.opponents = msg.robots

    def update_ball(self, msg: Ball):
        if self.on_positive_half:
            msg.position.x = -msg.position.x
            msg.position.y = -msg.position.y
        
        self.ball = msg
    
    def update_gc(self, msg: Referee):
        if (not(self.yellow_) and msg.blue_team_on_positive_half) or (self.yellow_ and not(msg.blue_team_on_positive_half)):
            self.on_positive_half = True
        else:
            self.on_positive_half = False
        self.gc = msg

    def ready(self):
        return len(self.allies) > 0 and len(self.opponents) > 0 and self.ball is not None
