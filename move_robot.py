from control.robot_motor_control import MotorControlV2
import argparse
from robot.catheter_robot_v2 import CatheterRobotV2
import matlab

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--has_cam", help="whether open camera or not", default=False, action="store_true")
    parser.add_argument("--cam", help="choose camera device id", type=int, default=4)
    
    
    args = parser.parse_args()

    DEVICENAME_1 = "/dev/ttyUSB0"  # Check which port is being used on your controller
    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    DEVICENAME_2 = "/dev/ttyUSB0"
    
    robot = CatheterRobotV2()

    
    motor_control = MotorControlV2(
        robot,
        [DEVICENAME_1, DEVICENAME_2],
        has_camera=args.has_cam,
        cam_id=args.cam,
    )

    motor_control.start_process()
    motor_control.end_process()
