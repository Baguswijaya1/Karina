import rclpy
import sense
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


        self.gps_sub = self.create_subscription(
            msg_type=GPSFix,
            topic='gps',
            callback=self.get_pos,
            qos_profile=10
        )

        self.rplidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic='lidar',
            callback=self.sense_obs,
            qos_profile=10
        )

    def get_pos(self, msg):
        self.posX = msg.latitude
        self.posY = msg.longitude
        self.heading = msg.track

    def sense_obs(self):
        pass

    def attractive_force(self):
        self.dist_to_target = sense.get_distance(self.targetX, self.targetY, self.posX, self.posY)
        return 0.5 * self.attgain * self.dist_to_target

    def repulsive_force(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = APF()
    rclpy.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
