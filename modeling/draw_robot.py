import numpy as np
import open3d as o3d
from catheter_robot import CatheterRobotV1
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import scipy.interpolate as interp
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def plot_segment(ax, points, radius, color=[0.7, 0.7, 0.7], circle_density=30):
    # points: 4x4xn
    num = points.shape[2]  # Number of points on the central curve
    # print(points[:, :, 0])
    x = points[0, 3, :]
    y = points[1, 3, :]
    z = points[2, 3, :]
    
    
    # Generate points around the central curve to form the tube surface
    theta = np.linspace(
        0, 2 * np.pi, circle_density
    )  # Circle points in the radial direction

    tangent_vectors = points[0:3, 2, :].transpose()
    # print(tangent_vectors)
    # print(tangent_vectors.shape)
    # Normal vectors: cross product of the tangent with an arbitrary vector (e.g., z-axis)
    normal_vectors = np.cross(tangent_vectors, np.array([0, 0, 1]))
    # Compute the norms of the normal_vectors
    norms = np.linalg.norm(normal_vectors, axis=1)

    # Avoid division by zero by checking for zero norms
    norms = np.where(norms == 0, 1, norms)  # Replace zero norms with 1

    # Normalize the normal_vectors (divide by norms)
    normal_vectors /= norms[:, np.newaxis]

    
    # Binormal vectors: cross product of tangent and normal
    binormal_vectors = np.cross(tangent_vectors, normal_vectors)

    # Create the tube surface by sweeping circles around the central curve
    # tube_points = []
    X = np.zeros([num, circle_density])
    Y = np.zeros([num, circle_density])
    Z = np.zeros([num, circle_density])
    
    # Loop over the central curve points and generate circles along the tube
    for i in range(num):
        # Parametrize the tube circle around the current point on the central curve
        circle_x = x[i] + radius * (
            normal_vectors[i, 0] * np.cos(theta)
            + binormal_vectors[i, 0] * np.sin(theta)
        )
        circle_y = y[i] + radius * (
            normal_vectors[i, 1] * np.cos(theta)
            + binormal_vectors[i, 1] * np.sin(theta)
        )
        circle_z = z[i] + radius * (
            normal_vectors[i, 2] * np.cos(theta)
            + binormal_vectors[i, 2] * np.sin(theta)
        )

        X[i, :] = circle_x
        Y[i, :] = circle_y
        Z[i, :] = circle_z
        
        # Store the circle points
        # tube_points.append(np.vstack((circle_x, circle_y, circle_z)).T)

    # Convert the list of points to a numpy array
    # tube_points = np.vstack(tube_points)
    # print(tube_points.shape)
    # ax.scatter(tube_points[:, 0], tube_points[:, 1], tube_points[:, 2], c=color, marker='o')
    ax.plot_surface(X, Y, Z, color=color)
    
def plot_CatheterRobotV1(robot: CatheterRobotV1, segments):
    # Five segments
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    plot_segment(ax, segments[0], robot.dc_out * 0.5, color=[102 / 255.0, 51 / 255.0, 0])
    plot_segment(ax, segments[1], robot.dc_out * 0.5, color=[204 / 255.0, 102 / 255.0, 0])
    plot_segment(ax, segments[2], robot.dc_out * 0.5, color=[0, 0, 0])
    plot_segment(ax, segments[3], robot.dn_out * 0.5, color=[0.7, 0.7, 0.7])
    plot_segment(ax, segments[4], robot.dn_out * 0.5, color=[0, 0, 0])
    
    ax.axis('equal')
    plt.show()

# def add_robot(
#     robot: CatheterRobotV1,
#     segments,
#     colors=[
#         [102 / 255.0, 51 / 255.0, 0],
#         [204 / 255.0, 102 / 255.0, 0],
#         [0, 0, 0],
#         [0.7, 0.7, 0.7],
#         [0, 0, 0],
#     ],
# ):
#     # segments: list of 4x4xN numpy arrays of transformations
#     segment_meshs = []
#     for i in range(len(segments)):
#         segment = segments[i]
#         radius = robot.dc_out if i < 3 else robot.dn_out
#         color = colors[i]
#         segment_mesh = add_segment(segment, radius, color)
#         segment_meshs.append(segment_mesh)

#     return segment_meshs


# def add_segment(points, radius, color, circle_density=30):
#     # points: 4x4xN numpy array of transformations
#     # r: radius of the tube
#     # thickness: thickness of the tube
#     # alpha: transparency of the tube
#     # color: color of the tube

#     num = points.shape[2]  # Number of points on the central curve
#     # print(points[:, :, 0])
#     x = points[0, 3, :]
#     y = points[1, 3, :]
#     z = points[2, 3, :]

#     # Generate points around the central curve to form the tube surface
#     theta = np.linspace(
#         0, 2 * np.pi, circle_density
#     )  # Circle points in the radial direction

#     tangent_vectors = points[0:3, 2, :].transpose()
#     # print(tangent_vectors)
#     # print(tangent_vectors.shape)
#     # Normal vectors: cross product of the tangent with an arbitrary vector (e.g., z-axis)
#     normal_vectors = np.cross(tangent_vectors, np.array([0, 0, 1]))
#     normal_vectors /= np.linalg.norm(normal_vectors, axis=1)[
#         :, None
#     ]  # Normalize the normal vectors

#     # Binormal vectors: cross product of tangent and normal
#     binormal_vectors = np.cross(tangent_vectors, normal_vectors)

#     # Create the tube surface by sweeping circles around the central curve
#     tube_points = []
#     faces = []

#     # Loop over the central curve points and generate circles along the tube
#     for i in range(num):
#         # Parametrize the tube circle around the current point on the central curve
#         circle_x = x[i] + radius * (
#             normal_vectors[i, 0] * np.cos(theta)
#             + binormal_vectors[i, 0] * np.sin(theta)
#         )
#         circle_y = y[i] + radius * (
#             normal_vectors[i, 1] * np.cos(theta)
#             + binormal_vectors[i, 1] * np.sin(theta)
#         )
#         circle_z = z[i] + radius * (
#             normal_vectors[i, 2] * np.cos(theta)
#             + binormal_vectors[i, 2] * np.sin(theta)
#         )

#         # Store the circle points
#         tube_points.append(np.vstack((circle_x, circle_y, circle_z)).T)

#     # Convert the list of points to a numpy array
#     tube_points = np.vstack(tube_points)

#     # Create faces (triangles) between consecutive circles
#     num_points = len(theta)  # Number of points in each circle
#     for i in range(num - 1):
#         for j in range(num_points):
#             # Indices of the current circle and the next circle
#             p1 = i * num_points + j
#             p2 = ((i + 1) % num) * num_points + j
#             p3 = i * num_points + (j + 1) % num_points
#             p4 = ((i + 1) % num) * num_points + (j + 1) % num_points

#             # Add two triangles to form the quad face
#             faces.append([p1, p2, p3])
#             faces.append([p3, p2, p4])

#     # Convert faces to numpy array
#     faces = np.array(faces)

#     # Create TriangleMesh for the tube
#     tube_mesh = o3d.geometry.TriangleMesh()
#     tube_mesh.vertices = o3d.utility.Vector3dVector(tube_points)
#     tube_mesh.triangles = o3d.utility.Vector3iVector(faces)

#     # Optionally, compute the normals for better visualization
#     tube_mesh.compute_vertex_normals()

#     tube_mesh.paint_uniform_color(color)

#     return tube_mesh

if __name__ == "__main__":
    robot = CatheterRobotV1()
    robot.actuation = np.array([3, -3, 0, 0])
    shape, ini10_sol, res, results = robot.fk_shooting()

    print("Seg1 tip position:", results['p'][0][:, -1])
    print("Seg2 tip position:", results['p'][1][:, -1])
    print("Seg3 tip position:", results['p'][2][:, -1])
    print("Seg4 tip position:", results['p'][3][:, -1])
    print("Seg5 tip position:", results['p'][4][:, -1])
    print("Residuals:", res)
    # print(results['g'][0].shape)
    plot_CatheterRobotV1(robot, results['g'])
    
    