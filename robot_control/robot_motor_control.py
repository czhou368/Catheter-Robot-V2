import numpy as np
from robot_control.xbox_control import XboxController
from robot_control.xbox_control_pygame import XboxControllerBluetooth
from robot_control.motor_control import DynamixelMotor
from dynamixel_sdk import *
import math
import cv2

from robot_control.camera_control import MicroCamera

MAX_VELOCITY_LINEAR = 5  # mm/s
MIN_VELOCITY_LINEAR = -5
STEP_VELOCITY_LINEAR = 1
DEFAULT_VELOCITY_LINEAR = 2
LINEAR_VELOCITY_FACTOR = 1 / 5.0 * 60 / 0.229
#                           lead  min   rev/min

MAX_VELOCITY_ROTATION = 45  # deg/s
MIN_VELOCITY_ROTATION = -45
ROTATION_VELOCITY_FACTOR = 1 / 360.0 * 60 / 0.229 * 8 / 5
#                            degree   min  rev/min gear ration

SEGMENT_1_RADIUS = 2  # mm
MAX_VELOCITY_BENDING_1 = 30  # deg/s
MIN_VELOCITY_BENDING_1 = -30
BENDING_VELOCITY_FACTOR_1 = 1 / 2.0 * math.pi / 180.0 * SEGMENT_1_RADIUS * 60 / 0.229

SEGMENT_2_RADIUS = 1  # mm
MAX_VELOCITY_BENDING_2 = 30  # deg/s
MIN_VELOCITY_BENDING_2 = -30
BENDING_VELOCITY_FACTOR_2 = 1 / 2.0 * math.pi / 180.0 * SEGMENT_2_RADIUS * 60 / 0.229 * 2

def safe_add(value, add_value, mininum, maximum):

    if value + add_value < mininum:
        print("Exceed minimum %d, not changed!" % mininum)
        return mininum

    if value + add_value > maximum:
        print("Exceed maximum %d, not changed!" % maximum)
        return maximum

    return value + add_value

# Catheter Robot V1 has 4 DoFs
class MotorControlV1:
    def __init__(self, 
                 device_name="/dev/ttyUSB0",
                 bauld_rate=57600,
                 use_bluetooth=False,
                 xbox_refresh_rate=1000,
                 has_camera=True,
                 cam_id=4,
                 ): 
        
        self.has_camera = has_camera
        
        self.portHandler = PortHandler(device_name)
        
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            exit()

        # Set port baudrate
        if self.portHandler.setBaudRate(bauld_rate):
            print("Succeeded to change the baudrate\n-----------------------------------")
        else:
            print("Failed to change the baudrate")
            exit()
        
        if use_bluetooth:
            self.xbox = XboxControllerBluetooth(refreshRate=xbox_refresh_rate)
        else:
            self.xbox = XboxController(refreshRate=xbox_refresh_rate)
        
        # Initialize the motors
        self.bending_motor_1 = DynamixelMotor("X_SERIES", 1, self.portHandler, 1, 0, 200, 4096*2, reverse=True)
        self.bending_motor_2 = DynamixelMotor("X_SERIES", 2, self.portHandler, 1, 0, 200, 4096*2.5, reverse=True)
        self.rotation_motor = DynamixelMotor(
            "X_SERIES", 3, self.portHandler, 2, 0, 50, safety_limit=4096 / 2.0 / 5.0 * 8, reverse=True
        )
        self.linear_motor = DynamixelMotor(
            "X_SERIES",
            4,
            self.portHandler,
            2,
            int(DEFAULT_VELOCITY_LINEAR * LINEAR_VELOCITY_FACTOR),
            400,
            safety_limit=4096 * 50 / 5.0,
            reverse=True,
        )

        self.capture_num = 0
        self.cam_id = cam_id
    def end_process(self):

        
        self.xbox.join()
        
        if self.has_camera:
            self.camera.stop()
            self.camera.join()
            

        del self.linear_motor
        del self.rotation_motor
        del self.bending_motor_1
        del self.bending_motor_2
        self.portHandler.closePort()
        
        print("-----------------------------------")
        print("Xbox control ended! Port closed.")
        print("-----------------------------------")
    
    def reset_motors(self):
        
        self.linear_motor.disable_torque(print_message=False)
        self.linear_motor.set_to_velocity_control_mode(print_message=False)
        self.linear_motor.enable_torque(print_message=False)

        self.rotation_motor.disable_torque(print_message=False)
        self.rotation_motor.set_to_velocity_control_mode(print_message=False)
        self.rotation_motor.enable_torque(print_message=False)

        self.bending_motor_1.disable_torque(print_message=False)
        self.bending_motor_1.set_to_velocity_control_mode(print_message=False)
        self.bending_motor_1.enable_torque(print_message=False)

        self.bending_motor_2.disable_torque(print_message=False)
        self.bending_motor_2.set_to_velocity_control_mode(print_message=False)
        self.bending_motor_2.enable_torque(print_message=False)
    
    def start_process(self):
        
        self.reset_motors()
        
        print("-----------------------------------") 
        print("Motors are ready!")  
        print("-----------------------------------")

        if self.has_camera:
            self.camera = MicroCamera(dev_id=self.cam_id)
            self.camera.start()
            print("-----------------------------------") 
            print("Camera is ready!")  
            print("-----------------------------------")
            
        self.xbox.start()
        self.rotation_goal_velocity = 0
        
        print("-----------------------------------")
        print("Xbox controller is ready!.")
        print("-----------------------------------")
        
        print("-----------------------------------")
        print("You can start controlling the catheter robot now!")
        print("Use the left stick up and down to bend the 1st segment.")
        print("Use the right stick up and down to bend the 2nd segment.")
        print("Use the left trigger to rotate counter-clockwise.")
        print("Use the right trigger to rotate clockwise.")
        print("Press up and down to move the robot linearly.")
        
        if self.has_camera:
            print("Press right trigger button to capture image.")
            
        print("Press the MENU button to exit controlling.")
        print("-----------------------------------")
        
        while not self.xbox.end:
            
            # Segment 1 Bending control
            # ---------------------
            self.bending_motor_1.set_goal_velocity(
                round(
                    self.xbox.left_stick_y * MAX_VELOCITY_BENDING_1 * BENDING_VELOCITY_FACTOR_1
                )
            )

            # Segment 2 Bending control
            # ---------------------
            self.bending_motor_2.set_goal_velocity(
                round(
                    self.xbox.right_stick_y * MAX_VELOCITY_BENDING_2 * BENDING_VELOCITY_FACTOR_2
                )
            )

            # Rotation control
            # ---------------------
            rotation_goal_velocity = round(
                (-self.xbox.lb + self.xbox.rb) * MAX_VELOCITY_ROTATION / ROTATION_VELOCITY_FACTOR
            )

            if rotation_goal_velocity == 0:
                self.rotation_motor.disable_torque(print_message=False)
            else:
                self.rotation_motor.enable_torque(print_message=False)

            self.rotation_motor.set_goal_velocity(rotation_goal_velocity)

            # Camera control
            if self.xbox.capture and self.has_camera:
                self.camera.capture_frame()
                self.xbox.capture = False
            
            # Linear motion control
            # ---------------------
            if self.xbox.left:

                self.linear_motor.dxl_default_velocity = round(
                    safe_add(
                        self.linear_motor.dxl_default_velocity / LINEAR_VELOCITY_FACTOR,
                        -STEP_VELOCITY_LINEAR,
                        MIN_VELOCITY_LINEAR,
                        MAX_VELOCITY_LINEAR,
                    )
                    * LINEAR_VELOCITY_FACTOR
                )
                self.xbox.left = False
                print(
                    "Set linear motion velocity to %d mm/s"
                    % round(self.linear_motor.dxl_default_velocity / LINEAR_VELOCITY_FACTOR)
                )

            elif self.xbox.right:
                self.linear_motor.dxl_default_velocity = round(
                    safe_add(
                        self.linear_motor.dxl_default_velocity / LINEAR_VELOCITY_FACTOR,
                        STEP_VELOCITY_LINEAR,
                        MIN_VELOCITY_LINEAR,
                        MAX_VELOCITY_LINEAR,
                    )
                    * LINEAR_VELOCITY_FACTOR
                )
                self.xbox.right = False
                print(
                    "Set linear motion velocity to %d mm/s"
                    % round(self.linear_motor.dxl_default_velocity / LINEAR_VELOCITY_FACTOR)
                )

            if self.xbox.up:
                self.linear_motor.set_goal_velocity(
                    self.linear_motor.dxl_default_velocity
                )
            elif self.xbox.down:
                self.linear_motor.set_goal_velocity(
                    -self.linear_motor.dxl_default_velocity
                )
            else:
                self.linear_motor.set_goal_velocity(0)

            # Home all motors
            # ---------------------
            if self.xbox.left_stick_button:
                print("Home all motors? Press A to confirm, B to cancel.")
                while True:
                    if self.xbox.a:
                        self.linear_motor.home()
                        self.rotation_motor.home()
                        self.bending_motor_1.home()
                        self.bending_motor_2.home()
                        
                        print("Homing all motors, please wait for 5 seconds.")
                        time.sleep(5)

                        self.reset_motors()
                        
                        print("Motors homed, ready to move.")
                        break
                    if self.xbox.b:
                        print("Cancel homing all motors.")
                        break
                    time.sleep(0.1)
                    
            # Reset home position for all motors
            # ---------------------
            if self.xbox.right_stick_button:
                print(
                    "Reset home position for all motors? Press A to confirm, B to cancel."
                )
                while True:
                    if self.xbox.a:
                        self.linear_motor.reset_home()
                        self.rotation_motor.reset_home()
                        self.bending_motor_1.reset_home()
                        self.bending_motor_2.reset_home()

                        print("Reset home position for all motors.")

                        break
                    if self.xbox.b:

                        print("Cancel reset home position for all motors.")

                        break
                    time.sleep(0.1)


    def forward_kinematics(self):
        pass