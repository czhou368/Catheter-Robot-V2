import pygame
import time
# from motor_control import DynamixelMotor
import threading

class XboxControllerBluetooth(threading.Thread):
    def __init__(self, refreshRate=100):
        threading.Thread.__init__(self)
        self.refreshTime = 0  # absolute time when next refresh (read results from xboxdrv stdout pipe) is to occur
        self.refreshDelay = (
            1.0 / refreshRate
        )  # joystick refresh is to be performed 100 times per sec by default
        #
        
        pygame.init()
        pygame.joystick.init()
        num_joysticks = pygame.joystick.get_count()

        if num_joysticks > 0:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            print("Controller connected:", self.controller.get_name())
        else:
            print("No controller detected.")
        
        
        # self.left_stick_x = 0
        self.left_stick_y = 0
        self.last_left_stick_y = 0
        # self.right_stick_x = 0
        self.right_stick_y = 0
        self.last_right_stick_y = 0
        self.a = 0
        self.b = 0
        self.x = 0
        self.y = 0
        self.lb = 0
        self.last_lb = 0
        self.rb = 0
        self.last_rb = 0
        self.lt = 0
        self.rt = 0
        self.end = 0

        self.left_stick_button = 0
        self.right_stick_button = 0
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        self.events = None

        self.capture = False
        self.record = False
        self.back = False
        
    def run(self):
        while True:
            self.refresh()
    
    def refresh(self):
        if self.refreshTime < time.time():
            self.refreshTime = time.time() + self.refreshDelay

            self.events = pygame.event.get()

            for event in self.events:
                self.read_event(event)

    # Scale raw (-32768 to +32767) axis with deadzone correcion
    # Deadzone is +/- range of values to consider to be center stick (ie. 0.0)
    def axisScale(self, raw, deadzone, last_value=0, range=(-32768.0, 32767.0)):
        if raw >= 0:
            if abs(raw) < deadzone or (last_value - raw > range[1] / 100.0):
                return 0.0
            else:
                return (raw - deadzone) / (range[1] - deadzone)
        else:
            if abs(raw) < deadzone or (last_value - raw < range[1] / 100.0):
                return 0.0
            else:
                return (raw + deadzone) / (-range[0] + deadzone)

    def read_event(self, event:pygame.event.Event):
        

        if event.type == pygame.JOYAXISMOTION:
            # First Segment Bending
            if event.axis == 1:
                self.left_stick_y = -self.axisScale(
                    event.dict['value'], 0.1, last_value=self.last_left_stick_y, range=(-0.8, 1.0)
                )
                self.last_left_stick_y = event.dict['value']
                # print("Left Stick Y: ", self.left_stick_y)
                
            # Second Segment Bending
            elif event.axis == 3:
                self.right_stick_y = -self.axisScale(
                    event.dict['value'], 0.1, last_value=self.last_right_stick_y, range=(-0.8, 1.0)
                )
                self.last_right_stick_y = event.dict['value']
                # print("Right Stick Y: ", self.right_stick_y)
            
            # Left Trigger
            elif event.axis == 5:
                self.lb = self.axisScale(
                    event.dict['value'] + 1, 0.2, last_value=self.last_lb, range=(-2.0, 2.0)
                )
                self.last_lb = event.dict['value'] + 1
                # print("Left Trigger: ", self.lb)

            # RIght Trigger
            elif event.axis == 4:
                self.rb = self.axisScale(
                    event.dict['value'] + 1, 0.2, last_value=self.last_rb, range=(-2.0, 2.0)
                )
                self.last_rb = event.dict['value'] + 1
                # print("Right Trigger: ", self.rb)
        
        elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
            
            # print(event.dict['button'])
            if event.type == pygame.JOYBUTTONDOWN: button_value = 1
            else: button_value = 0
            
            # Button A
            if event.dict['button'] == 0:
                self.a = button_value
                # print("Button A: ", self.a)
            
            # Button B
            elif event.dict['button'] == 1:
                self.b = button_value
                # print("Button B: ", self.b)
            
            # Button X
            elif event.dict['button'] == 3:
                self.x = button_value
                # print("Button X: ", self.x) 
                
            # Button Y
            elif event.dict['button'] == 4:
                self.y = button_value
                # print("Button Y: ", self.y)
            
            # Left Stick Button
            elif event.dict['button'] == 13:
                self.left_stick_button = button_value

            # Right Stick Button
            elif event.dict['button'] == 14:
                self.right_stick_button = button_value
            
            # Button Menu
            elif event.dict['button'] == 11:
                self.end = button_value

            # Button Start
            elif event.dict['button'] == 7:
                self.capture = button_value
            
            # Button Back
            elif event.dict['button'] == 6:
                self.back = button_value
            
        if event.type == pygame.JOYHATMOTION:
            
            hat_value = event.dict['value']
            
            if hat_value[0] == 1:
                self.right = True
                self.left = False
            elif hat_value[0] == -1:
                self.right = False
                self.left = True
            elif hat_value[0] == 0:
                self.right = False
                self.left = False
            
            if hat_value[1] == 1:
                self.down = False
                self.up = True
            elif hat_value[1] == -1:
                self.down = True
                self.up = False
            elif hat_value[1] == 0:
                self.down = False
                self.up = False
        
            
if __name__ == "__main__":
    xbox = XboxControllerBluetooth()

    xbox.start()
    # xbox.run()
    # while True:
    #     # print(time.time())
    #     xbox.refresh()
