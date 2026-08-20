import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from sensor_msgs.msg import LaserScan
import numpy as np

class APF(Node):
    def __init__(self):
        super().__init__('APF')
        
        # control limits

        # nav data
        self.targetX = 0
        self.targetY = 0
        self.dist_to_target = 0

        # gains
        self.attgain = 1
        self.repgain = 1

        # gps data
        self.posX = 0
        self.posY = 0   

        # lidar data
        self.lidar_range = []
        self.lidar_angle = []
        self.lidar_data = []

        self.rplidar_subs = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.get_lidar_data,
            qos_profile=10
        )

        # self.gps_sub = self.create_subscription(
        #     msg_type=GPSFix,
        #     topic='gps',
        #     callback=self.get_pos,
        #     qos_profile=10
        # )

    # def get_pos(self, msg):
    #     self.posX = msg.latitude
    #     self.posY = msg.longitude
    #     self.heading = msg.track


    # def get_lidar_data(self, msg):
    #     self.lidar_range = msg.ranges
    #     lidar_size = np.size(self.lidar_range)
    #     self.lidar_angle = np.zeros(lidar_size)
    #     for i in range(lidar_size):
    #         self.lidar_angle[i] = i/(lidar_size/360)

    #     self.lidar_data = np.column_stack((self.lidar_angle, self.lidar_range))

    #     self.get_logger().info(f'lidar_data: {self.lidar_data}')
        

    # def attractive_force(self):
    #     self.dist_to_target = sense.get_distance(self.targetX, self.targetY, self.posX, self.posY)
    #     return 0.5 * self.attgain * self.dist_to_target

    # def repulsive_force(self):
    #     pass

def main(args=None):
    rclpy.init(args=args)
    node = APF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
