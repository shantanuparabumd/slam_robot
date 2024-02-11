import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

import matplotlib.pyplot as plt


class ScanOdomSubscriber(Node):
    def __init__(self):
        super().__init__('scan_odom_subscriber')
        
        # Define QoS profile for the scan subscription
        scan_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=10,  # Assuming a history depth of 10
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT
        )
        
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos_profile)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.scan_data = []
        self.odom_data = []
        self.fig, self.ax = plt.subplots()
        # self.ax.set_aspect('equal', 'box')
        self.ax.set_xlim(-10, 10)  # Set x-axis limit
        self.ax.set_ylim(-10, 10)  # Set y-axis limit
        
        self.ax.autoscale(False)
        plt.title("Triangular Robot with Heading")
        self.timer = self.create_timer(0.1, self.update_plot)

        self.fig.canvas.mpl_connect('close_event', self.on_close)
        
        
    def scan_callback(self, msg):
        self.scan_data.append(msg.ranges)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.odom_data.append((x, y, z, yaw))

    def quaternion_to_euler(self, x, y, z, w):
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = np.arcsin(2 * (w * y - z * x))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

    def plot_scan_data(self,scan_data, robot_pose):
        transformed_scan_data = []
        for i, range_scan in enumerate(scan_data):
            if range_scan < 3.5:  # Ignore values exceeding maximum range
                angle_scan = np.radians(i)  # Calculate angle for the index
                x_robot, y_robot, theta_robot = robot_pose
                x_scan = x_robot + range_scan * np.cos(angle_scan + theta_robot)
                y_scan = y_robot + range_scan * np.sin(angle_scan + theta_robot)
                transformed_scan_data.append((x_scan, y_scan))  # Append transformed data
        return transformed_scan_data

    def plot_robot(self, x, y, yaw):
        triangle_size = 0.5
        triangle_points = np.array([[0, -triangle_size / 2], [triangle_size, 0], [0, triangle_size / 2]])
        rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rotated_triangle_points = np.dot(triangle_points, rotation_matrix.T)
        rotated_triangle_points += np.array([x, y])
        self.ax.plot(rotated_triangle_points[:, 0], rotated_triangle_points[:, 1], color='green',linewidth=2)
    
    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(-10, 10)  # Set x-axis limit
        self.ax.set_ylim(-10, 10)  # Set y-axis limit
        
        self.ax.autoscale(False)
        if self.odom_data and self.scan_data:
            x, y, _, yaw = self.odom_data[-1]  # Get latest odom data
            all_transformed_scan_data = []
            for scan_range,odom in zip(self.scan_data,self.odom_data):
                nx, ny,_, nyaw = odom
                transformed_scan_data = self.plot_scan_data(scan_range, (nx, ny, nyaw))
                all_transformed_scan_data.extend(transformed_scan_data)
            
            if all_transformed_scan_data:
                x_scan, y_scan = zip(*all_transformed_scan_data)  # Unzip the coordinates
                self.ax.plot(x_scan, y_scan, 'ko')  # Plot scan data as black dots
            self.plot_robot(x, y, yaw)
        plt.pause(0.001)

    def on_close(self, event):
        # Save the figure when the window is closed
        self.fig.savefig('robot_scan_plot.png')
        plt.close()
        

def main(args=None):
    rclpy.init(args=args)
    node = ScanOdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
