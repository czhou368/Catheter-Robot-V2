import matlab.engine
import numpy as np
from draw_robot import plot_CatheterRobotV1
from catheter_robot import CatheterRobotV1

if __name__ == "__main__":

    # Start MATLAB engine
    eng = matlab.engine.start_matlab()

    eng.addpath("../matlab")
    eng.addpath("../matlab/math_utils")
    eng.addpath("../matlab/plot_utils")

    actuations = [4, -4, 0, 0]
    robot_parameter = [170, 50, 5, 40, 5, 5.3, 3.0, 2.8, 2.5]
    E = matlab.double([60e3, 2e3, 60e3, 2e3, 60e3])
    G = matlab.double([20e3, 1e3, 20e3, 1e3, 20e3])
    loads = np.zeros((3, 3))
    distributed_load_1 = matlab.double(np.zeros((3, 1)).tolist())
    distributed_load_2 = matlab.double(np.zeros((3, 1)).tolist())
    end_force = matlab.double(np.zeros((3, 1)).tolist())

    robot = CatheterRobotV1(actuation=actuations,
                            parameters=robot_parameter,
                            E=E,
                            G=G,
                            loads=loads)
    
    sol, results, res = eng.forward_kinematics(
        matlab.double(actuations),
        matlab.double(robot_parameter),
        matlab.double(E),
        matlab.double(G),
        matlab.double(loads[:, 0].tolist()),
        matlab.double(loads[:, 1].tolist()),
        matlab.double(loads[:, 2].tolist()),
        nargout=3,
    )
    
    points = []
    
    for frames in results['g']:
        points.append(np.array(frames))
    
    plot_CatheterRobotV1(robot, points)

