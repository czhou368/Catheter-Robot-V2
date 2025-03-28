import numpy as np

def zigzag(matrix):
    zigzag = np.array([row[::-1] if i % 2 else row for i, row in enumerate(matrix)]).flatten()
    return zigzag

if __name__ == "__main__":
    
    actuation_limit_1 = 5
    actuation_limit_2 = 5
    
    number = 5
    
    # actuations_1 = np.random.uniform(-actuation_limit_1, actuation_limit_1, (number, 1))
    # actuations_2 = np.random.uniform(-actuation_limit_2, actuation_limit_2, (number, 1))
    
    actuations_1 = np.linspace(0, actuation_limit_1, number)
    actuations_2 = np.linspace(0, actuation_limit_2, number)
    
    # Create a 2D grid
    X, Y = np.meshgrid(actuations_1, actuations_2)
    
    actuations_first = np.column_stack([zigzag(X), -zigzag(X), zigzag(Y), -zigzag(Y)])
    actuations_second = np.column_stack([-zigzag(X), zigzag(X), zigzag(Y), -zigzag(Y)])
    actuations_third = np.column_stack([-zigzag(X), zigzag(X), -zigzag(Y), zigzag(Y)])
    actuations_fourth = np.column_stack([-zigzag(X), zigzag(X), zigzag(Y), -zigzag(Y)])
    
    # actuations = np.concatenate((actuations_1, -actuations_1, actuations_2, -actuations_2), axis=1)
    
    actuations = np.concatenate((actuations_first, actuations_second, actuations_third, actuations_fourth), axis=0)
    
    np.savetxt('actuations.txt', actuations, fmt='%f')