import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_3d_points(points):
    # Extract x, y, z coordinates from the list of points
    x_vals = [point[0] for point in points]
    y_vals = [point[1] for point in points]
    z_vals = [point[2] for point in points]

    # Create a figure
    fig = plt.figure()
    
    # Add 3D axes
    ax = fig.add_subplot(111, projection='3d')

    # Scatter plot the points
    ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')

    # Set labels for the axes
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # Show the plot
    plt.show()

# Example usage
# List of 3D points
# Define the list to store points as lists
points = [[0.025, -0.054, -0.19],
    [0.037921, -0.054, -0.198695],
    [0.0626794, -0.054, -0.205769],
    [0.0883857, -0.054, -0.212786],
    [0.107712, -0.054, -0.220283],
    [0.116417, -0.054, -0.228098],
    [0.11287, -0.054, -0.235662],
    [0.0975779, -0.054, -0.242233],
    [0.0727096, -0.054, -0.24709],
    [0.0416208, -0.054, -0.24967],
    [0.00837923, -0.054, -0.24967],
    [-0.0227096, -0.054, -0.24709],
    [-0.0475779, -0.054, -0.242233],
    [-0.0628698, -0.054, -0.235662],
    [-0.0664167, -0.054, -0.228098],
    [-0.057712, -0.054, -0.220283],
    [-0.0383857, -0.054, -0.212786],
    [-0.0126794, -0.054, -0.205769],
    [0.012079, -0.054, -0.198695],
    [0.025, -0.054, -0.19]]





plot_3d_points(points)
