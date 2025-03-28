from robot_control.robot_motor_control import MotorControlTrajV1
from modeling.catheter_robot import CatheterRobotV1
import numpy as np

if __name__ == "__main__":
    
    input("PLEASE ENSURE THE ROBOT IS AT HOME POSITION!!! Press ctl+c to cancel. Press Enter to start the process...")
    
    DEVICENAME = "/dev/ttyUSB0"  # Check which port is being used on your controller
    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    robot = CatheterRobotV1(parameters=[172, 40, 10, 40, 10, 5.2, 3, 4.0, 2.8])
    
    actuation_traj = np.loadtxt("actuations.txt", dtype=float)
    
    motor_control_traj = MotorControlTrajV1(
        robot,
        actuation_traj,
        DEVICENAME,
        has_camera=False,
        cam_id=4,
    )

    motor_control_traj.start_process()
    motor_control_traj.end_process()
