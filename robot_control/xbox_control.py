from inputs import get_gamepad
import time
# from motor_control import DynamixelMotor
import threading

class XboxController(threading.Thread):
    def __init__(self, refreshRate=100):
        threading.Thread.__init__(self)
        self.refreshTime = 0  # absolute time when next refresh (read results from xboxdrv stdout pipe) is to occur
        self.refreshDelay = (
            1.0 / refreshRate
        )  # joystick refresh is to be performed 30 times per sec by default
        #
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
        
        # self._stop = threading.Event()
        
    def run(self):
        while not self.end:
            self.refresh()
    
    def refresh(self):
        if self.refreshTime < time.time():
            self.refreshTime = time.time() + self.refreshDelay

            self.events = get_gamepad()

            for event in self.events:
                self.read_event(event)

    # Scale raw (-32768 to +32767) axis with deadzone correcion
    # Deadzone is +/- range of values to consider to be center stick (ie. 0.0)
    def axisScale(self, raw, deadzone, last_value=0, range=(-32768.0, 32767.0)):
        if raw >= 0:
            if abs(raw) < deadzone or (last_value - raw > range[1] / 200):
                return 0.0
            else:
                return (raw - deadzone) / (range[1] - deadzone)
        else:
            if abs(raw) < deadzone or (last_value - raw < range[1] / 200):
                return 0.0
            else:
                return (raw + deadzone) / (-range[0] + deadzone)

    def read_event(self, event):
        # if event.ev_type == "Key":
        #     print(event.ev_type, event.code, event.state)

        if event.ev_type == "Absolute":

            # Change Linear Motion Speed
            if event.code == "ABS_HAT0X":
                if event.state == -1:
                    self.right = False
                    self.left = True
                elif event.state == 1:
                    self.right = True
                    self.left = False
                elif event.state == 0:
                    self.right = False
                    self.left = False
            
            # Control Linear Motion
            if event.code == "ABS_HAT0Y":
                if event.state == -1:
                    self.down = False
                    self.up = True
                elif event.state == 1:
                    self.down = True
                    self.up = False
                elif event.state == 0:
                    self.down = False
                    self.up = False
            
            # Counterclockwise Rotation Motion
            elif event.code == "ABS_Z":
                self.lb = self.axisScale(
                    event.state, 200, last_value=self.last_lb, range=(0, 1023)
                )

                self.last_lb = event.state
                # print("Rotation speed: ", -self.lb)

            # Clockwise Rotation Motion
            elif event.code == "ABS_RZ":
                self.rb = self.axisScale(
                    event.state, 200, last_value=self.last_rb, range=(0, 1023)
                )

                self.last_rb = event.state
                # print("Rotation speed: ", self.rb)

            # Fisrt Segment Bending
            elif event.code == "ABS_Y":
                self.left_stick_y = -self.axisScale(
                    event.state, 4000, last_value=self.last_left_stick_y
                )
                self.last_left_stick_y = event.state
                # print("Left Stick Y: ", self.left_stick_y)

            # Second Segment Bending
            elif event.code == "ABS_RY":
                self.right_stick_y = -self.axisScale(
                    event.state, 4000, last_value=self.last_right_stick_y
                )
                self.last_right_stick_y = event.state
                # print("Right Stick Y: ", self.right_stick_y)
        
        elif event.ev_type == "Key":
            # Button A
            if event.code == "BTN_SOUTH":
                self.a = event.state
                # print("Button A: ", self.a)
            
            # Button B
            elif event.code == "BTN_EAST":
                self.b = event.state
                # print("Button B: ", self.b)
            
            # Button X
            elif event.code == "BTN_NORTH":
                self.x = event.state
                # print("Button X: ", self.x) 
                
            # Button Y
            elif event.code == "BTN_WEST":
                self.y = event.state
                # print("Button Y: ", self.y)
            
            elif event.code == "BTN_THUMBL":
                self.left_stick_button = event.state
                
            elif event.code == "BTN_THUMBR":
                self.right_stick_button = event.state
            
            elif event.code == "BTN_START":
                self.end = event.state
                
            elif event.code == "BTN_TR":
                self.capture = event.state
                
            elif event.code == "BTN_TL":
                self.record = event.state
    
    # # function using _stop function
    # def stop(self):
    #     self._stop.set()

    # def stopped(self):
    #     return self._stop.isSet()
    
if __name__ == "__main__":
    xbox = XboxController()

    xbox.start()
    # xbox.run()
