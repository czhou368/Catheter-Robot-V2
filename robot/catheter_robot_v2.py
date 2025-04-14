import numpy as np
import math
# from scipy.integrate import solve_ivp
# from scipy.optimize import fsolve


# Create a catheter robot v2 object


class CatheterRobotV2:
    def __init__(self, 
                 # parameters,
                 actuations=np.array([0, 0, 0, 0, 0, 0, 0, 0]),
                 ):
        # acuations: [bending_11, bending_12, rotation_1, insertion_1, bending_21, bending_22, rotation_2, insertion_2]
        # parameters: [todo]
        
        # self.parameters = parameters
        self.actuations = actuations
        
        


if __name__ == "__main__":
    robot = CatheterRobotV2()
    
