import numpy as np

def generate_linspaced_coordinates(x_limit, y_limit, z_limit, num_points_x, num_points_y, num_points_z):
    # Generate linspaced values for each axis
    x_values = np.linspace(0.1, x_limit, num_points_x)
    y_values = np.linspace(-y_limit, y_limit, num_points_y)
    z_values = np.linspace(0.18, z_limit, num_points_z)

    # Generate the grid of [x, y, z] points using meshgrid
    xx, yy, zz = np.meshgrid(x_values, y_values, z_values)

    # Stack the coordinates and reshape into a list of [x, y, z]
    coordinates = np.vstack([xx.ravel(), yy.ravel(), zz.ravel()]).T

    return coordinates

# Example usage:
x_limit = 0.2  # Define limits for x axis
y_limit = 0.2  # Define limits for y axis
z_limit = 0.3  # Define limits for z axis
num_points_x = 3  # Number of points along the x axis
num_points_y = 3  # Number of points along the y axis
num_points_z = 2  # Number of points along the z axis

coordinates = generate_linspaced_coordinates(x_limit, y_limit, z_limit, num_points_x, num_points_y, num_points_z)

print("Generated coordinates:")
print(coordinates)
