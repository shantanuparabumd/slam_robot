import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


# Function to convert quaternion to roll, pitch, and yaw
def quaternion_to_euler(x, y, z, w):
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return roll, pitch, yaw

# Load odom data from the text file
def load_odom_data(filename):
    odom_data = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            x, y, z = map(float, parts[:3])
            quat_x, quat_y, quat_z, quat_w = map(float, parts[3:])
            roll, pitch, yaw = quaternion_to_euler(quat_x, quat_y, quat_z, quat_w)
            odom_data.append((x, y, z, roll, pitch, yaw))
    return odom_data

# Function to plot the triangular robot
def plot_robot(ax, x, y, yaw):
    triangle_size = 0.2
    triangle_points = np.array([[0, -triangle_size / 2], [triangle_size, 0], [0, triangle_size / 2]])
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rotated_triangle_points = np.dot(triangle_points, rotation_matrix.T)
    rotated_triangle_points += np.array([x, y])
    ax.plot(rotated_triangle_points[:, 0], rotated_triangle_points[:, 1], 'b-')

# Create plot with slider
def create_plot_with_slider(odom_data):
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25)
    ax.set_aspect('equal', 'box')
    plt.title("Triangular Robot with Heading")

    ax_time = plt.axes([0.1, 0.1, 0.65, 0.03])
    slider = Slider(ax_time, 'Time Frame', 0, len(odom_data) - 1, valinit=0, valfmt='%0.0f')

    def update(val):
        time_frame = int(slider.val)
        ax.clear()
        ax.set_aspect('equal', 'box')
        plt.title("Triangular Robot with Heading")
        x, y, _, _, _, yaw = odom_data[time_frame]
        plot_robot(ax, x, y, yaw)
        ax.set_xlim(-5, 5)  # Adjust x-axis limits as needed
        ax.set_ylim(-5, 5)  # Adjust y-axis limits as needed
        plt.draw()

    slider.on_changed(update)

    plt.show()

if __name__ == "__main__":
    filename = "odom_clean_data.txt"
    odom_data = load_odom_data(filename)
    create_plot_with_slider(odom_data)
