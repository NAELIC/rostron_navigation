import json, os, pygame, time
import matplotlib.pyplot as plt

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action import ActionClient
from rostron_interfaces.action import Behavior
from rostron_interfaces.msg import Order, Orders, Hardware
from geometry_msgs.msg import Twist
from rostron_interfaces.msg import Commands, Command, Hardware, Robot
from rostron_ia_ms.strategies.control import Control
from rostron_ia_ms.strategies.go_to import GoTo
# from controller_movements import Controller
from rostron_navigation.main import RobotNavigation
from rostron_navigation.primitive.move_to import MoveTo
from rostron_utils.world import World

speed_multiplier = 10

class Joysticks(Node):
    ################################# LOAD UP A BASIC WINDOW #################################
    def __init__(self, id: int) -> None :
        self.id = id
        # self.publisher = World().node_.create_publisher(Orders, 'joysticks', 1)

        pygame.init()
        DISPLAY_W, DISPLAY_H = 960, 570
        canvas = pygame.Surface((DISPLAY_W,DISPLAY_H))
        window = pygame.display.set_mode(((DISPLAY_W,DISPLAY_H)))
        running = True
        player = pygame.Rect(DISPLAY_W/2, DISPLAY_H/2, 40,40)
        LEFT, RIGHT, UP, DOWN = False, False, False, False
        clock = pygame.time.Clock()
        pygame.display.set_caption("Joysticks Movement")
        color = 0

        # World.init(self)
        # self.robot_position = (World().allies[self.id].pose.position.x,
        #                     World().allies[self.id].pose.position.y)
        # self.current_location=(self.robot_position, time.time())
        self.speed_over_time=[[0.0, 0.0]] #  [[speed,time],...]
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
        analog_keys = {0:0, 1:0, 2:0, 3:0, 4:-1, 5:-1}

        # START OF GAME LOOP
        while running:
            ################################# CHECK PLAYER INPUT #################################
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    self.plot_speed_over_time()
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
                    if event.button == button_keys['circle']:
                        UP, DOWN, LEFT, RIGHT = False, False, False, False  
                        print("Stop!")

                # HANDLES BUTTON RELEASES
                if event.type == pygame.JOYBUTTONUP:
                    if event.button == button_keys['left_arrow']:
                        LEFT = False
                    if event.button == button_keys['right_arrow']:
                        RIGHT = False
                    if event.button == button_keys['down_arrow']:
                        DOWN = False
                    if event.button == button_keys['up_arrow']:
                        UP = False

                # HANDLES ANALOG INPUTS
                if event.type == pygame.JOYAXISMOTION:
                    analog_keys[event.axis] = event.value
                    # os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: ' + str(speed_multiplier*analog_keys[0]) + ', y: ' + str(speed_multiplier*analog_keys[1]) + '}, angular: { z: 0.0 }}}]}"')
                    # print(analog_keys) 
                    # Horizontal Analog
                    if abs(analog_keys[0]) > 0.5:
                        if analog_keys[0] < -0.7:
                            LEFT = True
                        else:
                            LEFT = False
                        if analog_keys[0] > 0.7:
                            RIGHT = True
                        else:
                            RIGHT = False
                    # Vertical Analog
                    if abs(analog_keys[1]) > 0.5:
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

                if event.type == pygame.JOYHATMOTION: # Doesn't work yet
                    """ Control with the arrows of the controller """

                    # get the position of a joystick hat
                    # get_hat(hat_number) -> x, y
                    # Returns the current position of a position hat. The position is given as two values representing the x and y position for the hat. 
                    # (-1, 0) means left; 
                    # (1, 0) means right;
                    # (0, 1) means up; 

                    hats = joystick.get_numhats()
                    for i in range(hats):
                        hat = joystick.get_hat(i)

                    if hat == (-1, 0):
                        LEFT = True
                        print("Go left!")
                        os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: 0.0, y: 1 }, angular: { z: 0.0 }}}]}"')
                    elif hat == (1, 0):
                        RIGHT = True
                        print("Go right!")
                        os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: 0.0, y: -1 }, angular: { z: 0.0 }}}]}"')
                    elif hat == (0, 1):
                        UP = True
                        print("Go up!")
                        os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: 1, y: 0 }, angular: { z: 0.0 }}}]}"')
                    elif hat == (0, -1):
                        DOWN = True
                        print("Go down!")
                        os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: -1, y: 0 }, angular: { z: 0.0 }}}]}"')

            # Handle Player movement
            if LEFT:
                # player.x -= 5
                print("Go left!")
                # print("Coordonnées : " + str(player.x) + " " + str(player.y))
                os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: 0.0, y: ' + str(speed_multiplier*(-analog_keys[0])) + ' }, angular: { z: 0.0 }}}]}"')
            if RIGHT:
                # player.x += 5
                print("Go right!")
                # print("Coordonnées : " + str(player.x) + " " + str(player.y))
                os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: 0.0, y: '  + str(speed_multiplier*(-analog_keys[0])) + ' }, angular: { z: 0.0 }}}]}"')
                
            if UP:
                # player.y -= 5
                print("Go up!")
                # print("Coordonnées : " + str(player.x) + " " + str(player.y))
                os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: ' + str(speed_multiplier*(-analog_keys[1])) + ', y: 0 }, angular: { z: 0.0 }}}]}"')
                
            if DOWN:
                # player.y += 5
                print("Go down!")
                # print("Coordonnées : " + str(player.x) + " " + str(player.y))
                os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: ' + str(speed_multiplier*(-analog_keys[1])) + ', y: 0 }, angular: { z: 0.0 }}}]}"')
                
                ##########################################
                # control = Order()
                # control.id = self.id
                
                # vel_msg = Twist()
                # vel_msg.linear.x = speed_multiplier*(-analog_keys[1])
                # vel_msg.linear.y = 0.0
                # control.velocity = vel_msg
                
                # msg = Orders()
                # msg.orders.append(control)
                # self.publisher.publish(msg)
                
                # rclpy.spin_once(self.node) 
                ##########################################
            if (LEFT==False and RIGHT==False and UP==False and DOWN==False):
                # self.stop_robot()
                os.system('ros2 topic pub --once /yellow/commands rostron_interfaces/msg/Commands "{ commands : [{ id : ' + str(id) + ' , velocity: { linear: { x: 0.0, y: 0.0 }, angular: { z: 0.0 }}}]}"')
                
            if color < 0:
                color = 0
            elif color > 255:
                color = 255

            ################################# UPDATE WINDOW AND DISPLAY #################################
            canvas.fill((255,255,255))
            pygame.draw.rect(canvas, (0,0 + color,255), player)
            window.blit(canvas, (0,0))
            clock.tick(10)
            pygame.display.update()

    def stop_robot(self):
        """Give to the robot a nullable velocity"""     
        control = Order()
        control.id = self.id
        control.velocity = Twist()

        msg = Orders()
        msg.orders.append(control)
        self.publisher.publish(msg)
            
    def speed_checker(self):
        last_location=self.current_location
        self.current_location=(self.robot_position, time.time())
        distance = self.distance(last_location[0],self.current_location[0])
        tim = self.current_location[1]- last_location[1]
        speed = distance/tim
        self.speed_over_time.append([speed,self.speed_over_time[-1][1]+tim])

    def plot_speed_over_time(self):
        speed = [i[0] for i in self.speed_over_time]
        time  = [i[1] for i in self.speed_over_time]
        plt.plot(time, speed, color='blue')
        plt.xlabel('Time (in sec)')
        plt.ylabel('Speed (in m/s)')

        average_speed= round(sum(speed)/len(speed),1)
        total_time= round(time[-1],1)
        distance = round(sum(speed)/len(speed)*time[-1],1)
        title= "Robot %i Path | Distance: %.1lfm | Time: %.1lfs | Average Speed: %.1lfm/s" % (self.id, distance, total_time, average_speed)
        plt.title(title)
        plt.show()

def main():
    rclpy.init()
    node = Joysticks(5)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

main()