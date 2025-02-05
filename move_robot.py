from robot_control.catheter_robot import CatheterRobotV1
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam", help="choose camera device id", type=int, default=4)

    args = parser.parse_args()

    DEVICENAME = "/dev/ttyUSB0"  # Check which port is being used on your controller
    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    robot_parameters = [170, 50, 5, 40, 5, 4.0, 2.0]

    catheter_robot = CatheterRobotV1(
        robot_parameters,
        DEVICENAME,
        use_bluetooth=False,
        has_camera=True,
        cam_id=args.cam,
    )

    catheter_robot.start_process()
    catheter_robot.end_process()
