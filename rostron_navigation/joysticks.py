import json, os, pygame

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action import ActionClient
from rostron_interfaces.action import Behavior
from rostron_interfaces.msg import Commands, Command, Hardware, Robot
from rostron_ia_ms.strategies.control import Control
from rostron_ia_ms.strategies.go_to import GoTo
from controller_movements import Controller
from rostron_navigation.main import RobotNavigation
from rostron_utils.world import World

def create_move_to(x, y, theta):
    msg = Behavior.Goal()

    msg.name = "move_to"
    msg.params = json.dumps({"x": x, "y": y, "theta": theta})

    return msg

class Joysticks(Node):
    tasks = []
    ################################# LOAD UP A BASIC WINDOW #################################
    def __init__(self) -> None :
        pygame.init()
        ###
        self.client_ = ActionClient(World().node_, Behavior, f"robot_{id}/behavior")
        self.client_.wait_for_server()
        
        self.create_timer(0.16, self.update)

        self.declare_parameter('yellow', True)
        self.is_yellow = self.get_parameter('yellow').get_parameter_value().bool_value
        thisRobot = World().allies[0]
        World().init(self, self.is_yellow)
        # self.client_.send_goal_async(create_move_to(0.0+self.player.x, 0.0+self.player.y, 0.0))
        ###
        DISPLAY_W, DISPLAY_H = 960, 570
        canvas = pygame.Surface((DISPLAY_W,DISPLAY_H))
        window = pygame.display.set_mode(((DISPLAY_W,DISPLAY_H)))
        running = True
        player = pygame.Rect(DISPLAY_W/2, DISPLAY_H/2, 40,40)
        LEFT, RIGHT, UP, DOWN = False, False, False, False
        clock = pygame.time.Clock()
        pygame.display.set_caption("Joysticks Movement")
        color = 0
        ###########################################################################################

        #Initialize controller
        joysticks = []
        for i in range(pygame.joystick.get_count()):
            joysticks.append(pygame.joystick.Joystick(i))
        for joystick in joysticks:
            joystick.init()

        with open(os.path.join("ps4_keys.json"), 'r+') as file:
            button_keys = json.load(file)
        # 0: Left analog horizonal, 1: Left Analog Vertical, 2: Right Analog Horizontal
        # 3: Right Analog Vertical 4: Left Trigger, 5: Right Trigger
        analog_keys = {0:0, 1:0, 2:0, 3:0, 4:-1, 5: -1 }

        # START OF GAME LOOP
        while running:
            ################################# CHECK PLAYER INPUT #################################
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    ############### UPDATE SPRITE IF SPACE IS PRESSED #################################
                    pass

                # HANDLES BUTTON PRESSES
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == button_keys['left_arrow']:
                        LEFT = True
                    if event.button == button_keys['right_arrow']:
                        RIGHT = True
                    if event.button == button_keys['down_arrow']:
                        DOWN = True
                    if event.button == button_keys['up_arrow']:
                        UP = True
                # HANDLES BUTTON RELEASES
                if event.type == pygame.JOYBUTTONUP:
                    if event.button == button_keys['left_arrow']:
                        LEFT = False
                        # os.system('ros2 topic pub /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : 0, velocity: { linear: { x: 0.0, y: 0 }, angular: { z: 0.0 }}}]}"')
                    if event.button == button_keys['right_arrow']:
                        RIGHT = False
                        # os.system('ros2 topic pub /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : 0, velocity: { linear: { x: 0.0, y: 0 }, angular: { z: 0.0 }}}]}"')
                    if event.button == button_keys['down_arrow']:
                        DOWN = False
                        # os.system('ros2 topic pub /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : 0, velocity: { linear: { x: 0.0, y: 0 }, angular: { z: 0.0 }}}]}"')
                    if event.button == button_keys['up_arrow']:
                        UP = False
                        # os.system('ros2 topic pub /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : 0, velocity: { linear: { x: 0.0, y: 0 }, angular: { z: 0.0 }}}]}"')

                #HANDLES ANALOG INPUTS
                if event.type == pygame.JOYAXISMOTION:
                    analog_keys[event.axis] = event.value
                    # print(analog_keys)
                    # Horizontal Analog
                    if abs(analog_keys[0]) > 0.4:
                        if analog_keys[0] < -0.7:
                            LEFT = True
                            
                        else:
                            LEFT = False
                        if analog_keys[0] > 0.7:
                            RIGHT = True
                        else:
                            RIGHT = False
                    # Vertical Analog
                    if abs(analog_keys[1]) > 0.4:
                        if analog_keys[1] < -0.7:
                            UP = True
                        else:
                            UP = False
                        if analog_keys[1] > 0.7:
                            DOWN = True
                        else:
                            DOWN = False
                        # Triggers
                    if analog_keys[4] > 0:  # Left trigger
                        color += 2
                    if analog_keys[5] > 0:  # Right Trigger
                        color -= 2

            # Handle Player movement
            if LEFT:
                player.x -=5
                # print(World.allies[0].pose.position.x)
                print("Go left!")
                # self.tasks.append(Controller(0, World.allies[0].pose.position.x-0.5, 0.0, 0.0))
                self.tasks.append(GoTo(0, -0.5, 0.0, 0.0))
            if RIGHT:
                player.x += 5
                print("Go right!")
                
            if UP:
                player.y -= 5
                print("Go up!")
                
            if DOWN:
                player.y += 5
                print("Go down!")
                
            if color < 0:
                color = 0
            elif color > 255:
                color = 255

            ################################# UPDATE WINDOW AND DISPLAY #################################
            canvas.fill((255,255,255))
            pygame.draw.rect(canvas, (0,0 + color,255), player)
            window.blit(canvas, (0,0))
            clock.tick(60)
            pygame.display.update()

    def update(self):
        for t in self.tasks:
            t.update()

# def main():
#     rclpy.init()
#     node = Joysticks()
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# main()