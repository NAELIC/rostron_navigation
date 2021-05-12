import rclpy
from rclpy.node import Node

from rostron_interfaces.msg import Ball
from std_msgs.msg import String


class BallPosition(Node):
    poseBall = Ball()
    def __init__(self):
        super().__init__('ball_position')
        self.subscription = self.create_subscription(
            Ball,
            '/yellow/ball',
            self.listener_ball,
            1
        )
        self.subscription

    def listener_ball(self, ball):
        global poseBall
        poseBall = ball
        return ball

def main(args=None):
    rclpy.init(args=args)

    ball_position = BallPosition()

    rclpy.spin_once(ball_position)
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    print("on est là")
    return ball_position.poseBall


if __name__ == '__main__':
    main()
