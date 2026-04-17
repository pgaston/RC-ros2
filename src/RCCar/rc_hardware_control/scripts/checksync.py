import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SyncChecker(Node):
    def __init__(self):
        super().__init__('sync_checker')
        self.last_imu_stamp = None
        self.last_img_stamp = None

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # self.create_subscription(Imu, '/camera/imu', self.imu_callback, sensor_qos)

        self.last_img1_stamp = None 
        self.last_img2_stamp = None

        # self.create_subscription(Imu, '/camera/imu', self.imu_callback, 10)
        self.create_subscription(Image, '/camera/infra1/image_rect_raw', self.img1_callback, sensor_qos)
        self.create_subscription(Image, '/camera/infra2/image_rect_raw', self.img2_callback, sensor_qos)
        print("SyncChecker node started, listening to /camera/infra1/image_rect_raw and /camera/infra2/image_rect_raw")

    def imu_callback(self, msg):
        self.last_imu_stamp = msg.header.stamp
        self.print_diff()

    def img1_callback(self, msg):
        self.last_img1_stamp = msg.header.stamp
        self.print_diff()

    def img2_callback(self, msg):
        self.last_img2_stamp = msg.header.stamp
        self.print_diff()

    def print_diff(self):
        if self.last_imu_stamp and self.last_img_stamp:
            # imu_time = self.last_imu_stamp.sec + self.last_imu_stamp.nanosec * 1e-9
            img1_time = self.last_img1_stamp.sec + self.last_img1_stamp.nanosec * 1e-9
            img2_time = self.last_img2_stamp.sec + self.last_img2_stamp.nanosec * 1e-9
            self.get_logger().info(f'Time difference (s): {diff:.6f}')

def main(args=None):
    rclpy.init(args=args)
    node = SyncChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    