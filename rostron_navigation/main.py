import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rostron_interfaces.action import MoveTo
from rostron_utils.world import World


from .primitive.move_to import MoveToPrimitive

from rostron_ia_ms.strategies.goalkeeper import GoalKeeper
from rostron_ia_ms.strategies.striker import Striker
from rostron_ia_ms.strategies.dribbler import Dribbler



class Navigation(Node):

    def __init__(self, id: int) -> None:
        super().__init__('navigation')
        self.id = id
        World().init(self)

        # Create all action and service for the navigation system.
        self.get_logger().info(
            'Init all actions and service for navigation')

        if self.id == 5:
            self.create_goalkeeper()
        elif self.id == 4:
            self.create_dribbler()
        else :
            self.create_move_to()
        
        

        self.get_logger().info(
            'Finish to init all action and service')

    def create_move_to(self):
        self.get_logger().info(
            'Init MoveTo primitive for the robot %d' % self.id)
        mt = MoveToPrimitive(self.id)

        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            mt.handler_callback)

    def create_goalkeeper(self):
        goal = GoalKeeper(self.id)
        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            goal.update_goal_keeper)
   
    def create_striker(self):
        striker= Striker(self.id)
        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            striker.move_to_ball_and_kick)
    
    def create_dribbler(self):
        dribbler= Dribbler(self.id)
        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            dribbler.move_to_ball_and_dribble)

def main(args=None):
    rclpy.init(args=args)
    id = 0
    navigation = Navigation(id)
    # waitable = WorldWaitable(navigation)
    # navigation.add_waitable(WorldWaitable)

    # navigation.get_logger().info('test')
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

def robot1(args=None):
    rclpy.init(args=args)
    id = 1
    navigation = Navigation(id)
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

def robot2(args=None):
    rclpy.init(args=args)
    id = 2
    navigation = Navigation(id)
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

def robot3(args=None):
    rclpy.init(args=args)
    print("Allies Main:", World().allies)
    id = 3
    navigation = Navigation(id)
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

def robot4(args=None):
    rclpy.init(args=args)
    id = 4
    navigation = Navigation(id)
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

def robot5(args=None):
    rclpy.init(args=args)
    id = 5
    navigation = Navigation(id)
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
