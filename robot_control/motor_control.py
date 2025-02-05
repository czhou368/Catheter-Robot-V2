import os

if os.name == "nt":
    import msvcrt

    def getch():
        return msvcrt.getch().decode()

else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ********* DYNAMIXEL Model definition *********
# ***** (Use only one definition at a time) *****
MY_DXL = "X_SERIES"  # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


class DynamixelMotor:
    def __init__(
        self,
        motor_series,
        motor_id,
        portHandler: PortHandler,
        protocol_version=2,
        default_velocity=0,
        velocity_limit=400,
        safety_limit=10000, # Maximum position difference for this motor from the home position, vital for bending motors!
        reverse=False,
    ):

        if motor_series == "X_SERIES" or motor_series == "MX_SERIES":
            self.ADDR_TORQUE_ENABLE = 64
            self.ADDR_GOAL_POSITION = 116
            self.ADDR_PRESENT_POSITION = 132
            # self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
            # self.DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
            self.ADDR_GOAL_VELOCITY = 104
            self.ADDR_PRESENT_VELOCITY = 128
            # self.DXL_MINIMUM_VELOCITY_VALUE  = 0         # Refer to the Minimum Velocity Limit of product eManual
            self.ADDR_OPERATING_MODE = 11
            self.ADDR_VELOCITY_LIMIT = 44
            # self.BAUDRATE                    = 57600
            self.ADDR_DRIVE_MODE = 10
            self.ADDR_PROFILE_VELOCITY = 112
        # elif motor_series == 'PRO_SERIES':
        #     self.ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
        #     self.ADDR_GOAL_POSITION          = 596
        #     self.ADDR_PRESENT_POSITION       = 611
        #     self.DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
        #     self.DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
        #     self.BAUDRATE                    = 57600
        # elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
        #     self.ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
        #     self.ADDR_GOAL_POSITION          = 564
        #     self.ADDR_PRESENT_POSITION       = 580
        #     self.DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
        #     self.DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
        #     self.BAUDRATE                    = 57600
        # elif MY_DXL == 'XL320':
        #     self.ADDR_TORQUE_ENABLE          = 24
        #     self.ADDR_GOAL_POSITION          = 30
        #     self.ADDR_PRESENT_POSITION       = 37
        #     self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
        #     self.DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
        #     self.BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps
        else:
            print("Invalid motor series!")
            quit()

        # Protocol version
        self.PROTOCOL_VERSION = (
            protocol_version  # See which protocol version is used in the Dynamixel
        )

        # Default setting
        self.DXL_ID = motor_id  # Dynamixel ID : 1

        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        # self.DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

        # dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        # self.portHandler = PortHandler(self.DEVICENAME)
        self.portHandler = portHandler

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if reverse:
            self.set_drive_mode(1)
        
        self.dxl_present_position = 0
        self.get_current_position()
        self.dxl_present_velocity = 0
        self.get_current_velocity()

        self.dxl_default_velocity = default_velocity
        self.dxl_goal_velocity = 0
        # self.set_goal_velocity(self.dxl_goal_velocity)

        self.dxl_velocity_limit = velocity_limit
        #

        self.dxl_home_position = self.dxl_present_position
        self.safety_limit = safety_limit
        
    def set_drive_mode(self, drive_mode):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_DRIVE_MODE, drive_mode
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when setting drive mode for motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when setting drive mode for motor %d" % self.DXL_ID
            )
    
    def motor_limit_check(self):
        self.get_current_position(print_message=False)
        
        if self.dxl_present_position >= self.dxl_home_position + self.safety_limit:
            return 1 # Reached Maximum
        elif self.dxl_present_position <= self.dxl_home_position - self.safety_limit:
            return 2 # Reached Minimum
        else:
            return 0 # In the range
        
    def enable_torque(self, print_message=True):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when enabling torque for motor %d" % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print("Error occurred when enabling torque for motor %d" % self.DXL_ID)
        elif print_message:
            print("Torque enabled for motor %d" % self.DXL_ID)

    def disable_torque(self, print_message=True):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when disabling torque for motor %d" % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print("Error occurred when disabling torque for motor %d" % self.DXL_ID)
        elif print_message:
            print("Torque disabled for motor %d" % self.DXL_ID)

    def get_current_position(self, print_message=True):
        self.dxl_present_position, dxl_comm_result, dxl_error = (
            self.packetHandler.read4ByteTxRx(
                self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION
            )
        )
        
        if self.dxl_present_position > 2147483647:
            self.dxl_present_position = self.dxl_present_position - 4294967296
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when getting current position of motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when getting current position of motor %d" % self.DXL_ID
            )
        elif print_message:
            print(
                "Current position of motor %d is %d"
                % (self.DXL_ID, self.dxl_present_position)
            )

    def get_current_velocity(self):
        self.dxl_present_velocity, dxl_comm_result, dxl_error = (
            self.packetHandler.read4ByteTxRx(
                self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY
            )
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when getting current velocity of motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when getting current velocity of motor %d" % self.DXL_ID
            )
        # else:
        #     print("Current velocity of motor %d is %d" % (self.DXL_ID, self.dxl_present_velocity))

    def set_goal_velocity(self, goal_velocity):
        
        limit_check = self.motor_limit_check()

        if limit_check == 1 and goal_velocity >= 0:
            # print(self.dxl_present_position)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, 0
            )

            if self.DXL_ID == 3: self.disable_torque(print_message=False) # Motor 3 won't stop evenif given velocity 0
            
            print("Motor %d reached the maximum postion!" % self.DXL_ID)
        
        elif limit_check == 2 and goal_velocity <= 0:
            
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, 0
            )
            
            if self.DXL_ID == 3: self.disable_torque(print_message=False)
            
            print("Motor %d reached the minimum position!" % self.DXL_ID)
        
        else:
        
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, goal_velocity
                )
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when setting goal velocity for motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when setting goal velocity for motor %d" % self.DXL_ID
            )
        # else:
        # print("Goal velocity set to %d for motor %d" % (goal_velocity, self.DXL_ID))

    def set_goal_position(self, goal_position):
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, goal_position
            )
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when setting goal position for motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when setting goal position for motor %d" % self.DXL_ID
            )
        # else:
        # print("Goal position set to %d for motor %d" % (goal_position, self.DXL_ID))
    
    def set_to_velocity_control_mode(self, print_message=True):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, 1
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when setting velocity control mode for motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when setting velocity control mode for motor %d"
                % self.DXL_ID
            )
        else:
            if print_message:
                print("Set velocity control mode for motor %d" % self.DXL_ID)

            self.set_velocity_limit(self.dxl_velocity_limit)

    def set_to_extended_position_control_mode(self, print_message=True):
        # self.disable_torque()
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, 4
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when setting extended position control mode for motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when setting extended position control mode for motor %d"
                % self.DXL_ID
            )
        elif print_message:
            # self.enable_torque()
            print("Set extended position control mode for motor %d" % self.DXL_ID)
    
    def set_velocity_limit(self, velocity_limit):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_VELOCITY_LIMIT, velocity_limit
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when setting velocity limit for motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when setting velocity limit for motor %d" % self.DXL_ID
            )
        else:
            print(
                "Set velocity limit to %d for motor %d" % (velocity_limit, self.DXL_ID)
            )

    
    def set_velocity_profile(self, velocity_profile):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_PROFILE_VELOCITY, velocity_profile
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print(
                "Communication failed when setting velocity profile for motor %d"
                % self.DXL_ID
            )
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            print(
                "Error occurred when setting velocity profile for motor %d" % self.DXL_ID
            )
        
        
    def home(self):
        self.disable_torque(print_message=False)
        self.set_to_extended_position_control_mode(print_message=False)
        self.enable_torque(print_message=False)
        
        self.set_velocity_profile(self.dxl_velocity_limit)
        self.set_goal_position(self.dxl_home_position)
        
    def reset_home(self):
        self.get_current_position()
        self.dxl_home_position = self.dxl_present_position
        print("Home position set to %d for motor %d" % (self.dxl_home_position, self.DXL_ID))
        
    def __del__(self):
        # Close port
        self.disable_torque()
