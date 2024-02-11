import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        # Initialize subscribers
        self.cmdvel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdvel_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        # Define QoS profile for the scan subscription
        scan_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=10,  # Assuming a history depth of 10
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT
        )

        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            scan_qos_profile
        )

        # Open log files
        self.cmdvel_log = open('cmdvel_log.txt', 'w')
        self.odom_log = open('odom_log.txt', 'w')
        self.scan_log = open('scan_log.txt', 'w')

    def cmdvel_callback(self, msg):
        self.cmdvel_log.write(f'{msg}\n')

    def odom_callback(self, msg):
        self.odom_log.write(f'{msg}\n')

    def scan_callback(self, msg):
        self.scan_log.write(f'{msg}\n')

    def close_logs(self):
        self.cmdvel_log.close()
        self.odom_log.close()
        self.scan_log.close()

def main(args=None):
    rclpy.init(args=args)

    logger_node = LoggerNode()

    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass

    logger_node.close_logs()
    logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
