# Import the librairies 
import json, os, pygame, time, sys
import matplotlib.pyplot as plt
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rostron_interfaces.msg import Commands, Command, Hardware

speed_multiplier = 6.0

########################################################

##### TODO : Resolve the problem of the dribbler   #####

########################################################

class Joysticks(Node):
    def __init__(self, id: int):
        super().__init__('minimal_publisher')

        self.publisher = self.create_publisher(Commands,"/yellow/commands", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.id = id
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0

        self.speed_over_time = [[0.0, 0.0]] #  [[speed,time],...]

        self.EventHandler = threading.Thread(target = self.window, args = ())
        self.EventHandler.start()
        
    ################################# LOAD UP A BASIC WINDOW #################################
    def window(self):
        pygame.init()
        DISPLAY_W, DISPLAY_H = 960, 570
        canvas = pygame.Surface((DISPLAY_W,DISPLAY_H))
        window = pygame.display.set_mode(((DISPLAY_W,DISPLAY_H)))
        player = pygame.Rect(DISPLAY_W/2, DISPLAY_H/2, 40,40)
        clock = pygame.time.Clock()
        pygame.display.set_caption("Joysticks Movement")
        color = 0
        ###########################################################################################

        # Initialize controller
        joysticks = []
        for i in range(pygame.joystick.get_count()):
            joysticks.append(pygame.joystick.Joystick(i))
        for joystick in joysticks:
            joystick.init()

        with open(os.path.join("src/rostron_navigation/rostron_navigation/ps4_keys.json"), 'r+') as file:
            button_keys = json.load(file)
        # 0: Left analog horizonal, 1: Left Analog Vertical, 3: Right Analog Horizontal
        # 4: Right Analog Vertical 2: Left Trigger, 5: Right Trigger
        self.analog_keys = {0:0, 1:0, 2:0, 3:0, 4:-1, 5:-1}

        LEFT, RIGHT, UP, DOWN, ROTATE_LEFT, ROTATE_RIGHT, FLAT_KICK, CHIP_KICK, DRIBBLER = False, False, False, False, False, False, False, False, False
        canvas.fill((255,255,255))
        pygame.draw.rect(canvas, (0,0 + color,255), player)
        window.blit(canvas, (0,0))
        clock.tick(1)
        pygame.display.update()

        # START OF GAME LOOP
        running = True
        while running:
            ################################# CHECK PLAYER INPUT #################################
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    pygame.quit()
                    sys.exit()
                # For computer inputs 
                if event.type == pygame.KEYDOWN:
                    key_code = event.__dict__['scancode']
                    if key_code == 79:
                        RIGHT = True
                    if key_code == 80:
                        LEFT = True
                    if key_code == 81:
                        DOWN = True
                    if key_code == 82:
                        UP = True
                if event.type == pygame.KEYUP:
                    key_code = event.__dict__['scancode']
                    if key_code == 79:
                        RIGHT = False
                    if key_code == 80:
                        LEFT = False
                    if key_code == 81:
                        DOWN = False
                    if key_code == 82:
                        UP = False

                # HANDLES BUTTON PRESSES
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == button_keys['left_stick_click']:
                        UP = True
                    if event.button == button_keys['right_stick_click']:
                        DOWN = True
                    if event.button == button_keys['circle']:
                        FLAT_KICK = True
                    if event.button == button_keys['square']:
                        CHIP_KICK = True
                    if event.button == button_keys['triangle']:
                        if DRIBBLER == False :
                            DRIBBLER = True
                        else :
                            DRIBBLER = False
                    if event.button == button_keys['L1']:
                        print("Previous player!")
                        if self.id > 0 :
                            self.id -= 1
                        else :
                            self.id = 5
                        print("Player " + str(self.id) + " selected!")
                    if event.button == button_keys['R1']:
                        print("Next player!")
                        if self.id < 5 :
                            self.id += 1
                        else :
                            self.id = 0
                        print("Player " + str(self.id) + " selected!")
                    if event.button == button_keys['x']:
                        UP, DOWN, LEFT, RIGHT, ROTATE_LEFT, ROTATE_RIGHT, FLAT_KICK, CHIP_KICK, DRIBBLER = False, False, False, False, False, False, False, False, False  
                        print("Stop!")

                # HANDLES BUTTON RELEASES
                if event.type == pygame.JOYBUTTONUP:
                    if event.button == button_keys['left_stick_click']:
                        UP = False
                    if event.button == button_keys['right_stick_click']:
                        DOWN = False
                    if event.button == button_keys['circle']:
                        FLAT_KICK = False
                    if event.button == button_keys['square']:
                        CHIP_KICK = False

                # HANDLES ANALOG INPUTS
                if event.type == pygame.JOYAXISMOTION:
                    """ Control with the joysticks of the controller """
                    self.analog_keys[event.axis] = event.value
                    # Horizontal Analog
                    if abs(self.analog_keys[0]) > 0.5:
                        if self.analog_keys[0] < -0.7:
                            LEFT = True
                        else:
                            LEFT = False
                        if self.analog_keys[0] > 0.7:
                            RIGHT = True
                        else:
                            RIGHT = False
                    # Vertical Analog
                    if abs(self.analog_keys[1]) > 0.5:
                        if self.analog_keys[1] < -0.7:
                            UP = True
                        else:
                            UP = False
                        if self.analog_keys[1] > 0.7:
                            DOWN = True
                        else:
                            DOWN = False
                    # Triggers : Rotation
                    if self.analog_keys[2] > 0.5:  # Left trigger
                        ROTATE_LEFT = True
                    else :
                        ROTATE_LEFT = False
                    if self.analog_keys[5] > 0.5:  # Right Trigger
                        ROTATE_RIGHT = True
                    else :
                        ROTATE_RIGHT = False

                if event.type == pygame.JOYHATMOTION: 
                    """ Control with the arrows of the controller """

                    # get the position of a joystick hat
                    # get_hat(hat_number) -> x, y
                    # Returns the current position of a position hat. The position is given as two values representing the x and y position for the hat. 
                    # (-1, 0) means left 
                    # (1, 0) means right
                    # (0, 1) means up 

                    hats = joystick.get_numhats()
                    for i in range(hats):
                        hat = joystick.get_hat(i)

                    if hat == (-1, 0):
                        LEFT = True
                        print("Go left!")
                    elif hat == (1, 0):
                        RIGHT = True
                        print("Go right!")
                    elif hat == (0, 1):
                        UP = True
                        print("Go up!")
                    elif hat == (0, -1):
                        DOWN = True
                        print("Go down!")
                    else :
                        UP, DOWN, LEFT, RIGHT, ROTATE_LEFT, ROTATE_RIGHT = False, False, False, False, False, False  
                        
            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_z = 0.0

            # Handle Player movements & actions
            if LEFT:
                print("Go left!")
                self.vel_y += speed_multiplier * max(self.analog_keys[0], 1/speed_multiplier)
                if player.x > 0:
                    player.x -= 5

            if RIGHT:
                print("Go right!")
                self.vel_y -= speed_multiplier * max(self.analog_keys[0], 1/speed_multiplier)
                if player.x < DISPLAY_W - 40:
                    player.x += 5
    
            if UP:
                print("Go up!")
                self.vel_x += speed_multiplier * max(self.analog_keys[1], 1/speed_multiplier)
                if player.y > 0:
                    player.y -= 5

            if DOWN:
                print("Go down!")
                self.vel_x -= speed_multiplier * max(self.analog_keys[1], 1/speed_multiplier)
                if player.y < DISPLAY_H - 40:
                    player.y += 5

            if ROTATE_LEFT:
                print("Rotate left!")
                self.vel_z += speed_multiplier * max(self.analog_keys[2], 1/speed_multiplier) / 2
            
            if ROTATE_RIGHT:
                print("Rotate right!")
                self.vel_z -= speed_multiplier * max(self.analog_keys[5], 1/speed_multiplier) / 2
            
            if FLAT_KICK:
                print("Flat kick!")
                self.flat_kick()
            
            if CHIP_KICK:
                print("Chip kick!")
                self.chip_kick()
            
            if DRIBBLER:
                print("Dribbler activated!")
                self.catch_ball()
                
            ################################# UPDATE WINDOW AND DISPLAY AND TIMER #################################
            canvas.fill((255,255,255))
            pygame.draw.rect(canvas, (0,0 + color,255), player)
            window.blit(canvas, (0,0))
            clock.tick(10)
            pygame.display.update()

    """
    def plot_speed_over_time(self):
        speed = [i[0] for i in self.speed_over_time]
        time  = [i[1] for i in self.speed_over_time]
        plt.plot(time, speed, color='blue')
        plt.xlabel('Time (in sec)')
        plt.ylabel('Speed (in m/s)')

        average_speed = round(sum(speed)/len(speed), 1)
        total_time = round(time[-1], 1)

        title = "Robot %i | Time: %.1lfs | Average Speed: %.1lfm/s" % (self.id, total_time, average_speed)
        plt.title(title)
        plt.show()
    """

    def flat_kick(self):
        command = Command()
        command.id = self.id

        hardware = Hardware()
        hardware.kick_power = 1.0
        hardware.kick_type = Hardware.FLAT_KICK
        command.hardware = hardware

        msg = Commands()
        msg.commands.append(command)
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "{ commands : [{ id : %d , kick type: %s }]}"' % (msg.commands[0].id, hardware.kick_type))
    
    def chip_kick(self):
        command = Command()
        command.id = self.id

        hardware = Hardware()
        hardware.kick_power = 1.0
        hardware.kick_type = Hardware.CHIP_KICK
        command.hardware = hardware

        msg = Commands()
        msg.commands.append(command)
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "{ commands : [{ id : %d , kick type: %s }]}"' % (msg.commands[0].id, hardware.kick_type))

    def catch_ball(self):
        command = Command()
        command.id = self.id

        hardware = Hardware()
        hardware.kick_type = Hardware.NO_KICK
        hardware._spin_power = 1.0
        command.hardware = hardware

        msg = Commands()
        msg.commands.append(command)
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "{ commands : [{ id : %d , kick type: %s }]}"' % (msg.commands[0].id, hardware.kick_type))

    def timer_callback(self):
        control = Command()
        control.velocity = Twist()

        control.id = self.id
        control.velocity.linear.x = self.vel_x
        control.velocity.linear.y = self.vel_y
        control.velocity.angular.z = self.vel_z

        msg = Commands()
        msg.commands.append(control)
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "{ commands : [{ id : %d , velocity: { linear: { x: %d, y: %d}, angular: { z: %d }}}]}"' % (msg.commands[0].id, msg.commands[0].velocity.linear.x, msg.commands[0].velocity.linear.y, msg.commands[0].velocity.angular.z))

def main():
    rclpy.init()
    node = Joysticks(0)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()