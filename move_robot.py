from robot_control.robot_motor_control import MotorControlV1
import argparse
from modeling.catheter_robot import CatheterRobotV1
import matlab

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--has_cam", help="whether open camera or not", default=False, action="store_true")
    parser.add_argument("--cam", help="choose camera device id", type=int, default=4)
    
    
    args = parser.parse_args()

    DEVICENAME = "/dev/ttyUSB0"  # Check which port is being used on your controller
    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    robot = CatheterRobotV1(parameters=[172, 40, 10, 40, 10, 5.2, 3, 4.0, 2.8])
    
    # Start MATLAB engine
    eng = matlab.engine.start_matlab()

    eng.addpath("./matlab/ChangBot")
    eng.addpath("./matlab/ChangBot/mathUtil")
    eng.addpath("./matlab/ChangBot/plotFunc")
    
    
    motor_control = MotorControlV1(
        robot,
        eng,
        DEVICENAME,
        use_bluetooth=False,
        has_camera=args.has_cam,
        cam_id=args.cam,
    )

    motor_control.start_process()
    motor_control.end_process()
