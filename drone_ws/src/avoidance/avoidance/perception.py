import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_interface.msg import PolarScan
import numpy as np

class Perception(Node):
    def __init__(self):
        super().__init__('Perception')

        # lidar data
        self.lidar_range = []
        self.lidar_angle = []
        self.lidar_data = []

        self.range_min = 0
        self.range_max = 0

        self.rplidar_subs = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.get_lidar_data,
            qos_profile=10
        )

        self.range_and_angle_pubs = self.create_publisher(
            msg_type=PolarScan,
            topic='/lidar/rangeangle',
            qos_profile=10
        )

    # def get_lidar_data(self, msg):
    #     self.lidar_range = msg.ranges
    #     lidar_size = np.size(self.lidar_range)
    #     # self.get_logger().info(f'lidar size = {lidar_size}')
    #     self.lidar_angle = np.zeros(lidar_size)
    #     for i in range(lidar_size):
    #         self.lidar_angle[i] = i/(lidar_size/360)

    #     to_pub = PolarScan()
    #     to_pub.angle_deg = self.lidar_angle
    #     to_pub.range = self.lidar_range
    #     self.range_and_angle_pubs.publish(to_pub)


    #     # self.get_logger().info(f'lidar_data: {self.lidar_data}')

    # saran claude  
    def get_lidar_data(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size

        angles_rad = (msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment).astype(np.float32)
        angles_deg = np.degrees(angles_rad).astype(np.float32)
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        to_pub = PolarScan()
        to_pub.angle_deg = angles_deg
        to_pub.angle_rad = angles_rad
        to_pub.range = ranges
        self.range_and_angle_pubs.publish(to_pub)

def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
