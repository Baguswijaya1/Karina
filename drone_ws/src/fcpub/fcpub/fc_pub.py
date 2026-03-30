import rclpy
from rclpy.node import Node
import serial
from dataclasses import dataclass

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


@dataclass
class FrameV1:
    roll: float
    pitch: float
    yaw: float
    arm: bool

def parse_frame(line:str) -> FrameV1:
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 4:
        raise ValueError('Expected 4 values: roll,pitch,yaw, armStatus')
    
    roll = float(parts[0])
    pitch = float(parts[1])
    yaw = float(parts[2])
    arm_raw = parts[3].lower()
    if arm_raw in ("1", "true"):
        arm = True
    elif arm_raw in ("0", "false"):
        arm = False
    else:
        raise ValueError("arm must be 0/1")
    
    return FrameV1(roll=roll, pitch=pitch, yaw=yaw, arm=arm) 

class FCpub(Node):
    def __init__(self):
        super().__init__('FCpub')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.imu_pub = self.create_publisher(
            msg_type=Twist,
            topic='imu_data',
            qos_profile=10
        )

        self.arm_pub = self.create_publisher(
            msg_type=Bool,
            topic='arming_status',
            qos_profile=10
        )

        self.get_logger().info('node started')
        self.timer = self.create_timer(0.05, self.parse)

    def parse(self):
        
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return
        try:
            frame = parse_frame(line)

            # imu data
            imu_data = Twist()
            imu_data.angular.x = frame.roll
            imu_data.angular.y = frame.pitch
            imu_data.angular.z = frame.yaw
            self.imu_pub.publish(imu_data)

            # arming status
            armStatus = Bool()
            armStatus.data = frame.arm
            self.arm_pub.publish(armStatus)

            
        except Exception as e:
            self.get_logger().warn(f"Invalid data: {line} | error: {e}")
        

def main(args=None):
    rclpy.init(args=args)
    node = FCpub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()