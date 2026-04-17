import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthDiagnostic(Node):
    def __init__(self):
        super().__init__('depth_diagnostic')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.get_logger().info('Monitoring depth data... (Ctrl+C to stop)')

    def listener_callback(self, msg):
        # Convert buffer to numpy array (16-bit unsigned integers)
        depth_data = np.frombuffer(msg.data, dtype=np.uint16)
        
        # Filter for non-zero values
        valid_pixels = depth_data[depth_data > 0]
        
        total_pixels = len(depth_data)
        valid_count = len(valid_pixels)
        
        if valid_count > 0:
            avg_depth = np.mean(valid_pixels)
            # Typically 16UC1 is in millimeters
            self.get_logger().info(
                f"Valid Pixels: {valid_count}/{total_pixels} | Avg Depth: {avg_depth:.2f}mm"
            )
        else:
            self.get_logger().warn("Frame received, but 100% of pixels are ZERO.")

def main(args=None):
    rclpy.init(args=args)
    node = DepthDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()