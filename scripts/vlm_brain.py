import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np

from nano_llm import NanoLLM
from nano_llm.plugins import WebServer
import threading
import re

class EventGatedBrain(Node):
    def __init__(self):
        super().__init__('vlm_brain')
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_nvblox_map_image = None
        
        # 1. Image Subscribers (Camera & Nvblox Map)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 1)
        # Note: adjust the nvblox topic name if your robot publishes the OccupancyGrid elsewhere!
        self.create_subscription(OccupancyGrid, '/nvblox_node/map_slice', self.map_cb, 1)
        
        # 2. Navigation Publisher (Talks to Nav2)
        self.nav_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 3. Load the VLM Desktop
        self.model = NanoLLM.from_pretrained(
            "Efficient-Large-Model/VILA1.5-3b", 
            api='mlc',
            max_context_len=1024,
            system_prompt="You are a robot assistant. If you receive two visual inputs, the first is a camera image and the second is a top-down obstacle map from nvblox. If asked to move to a destination, output your reasoning based on the map, and then end the message with [NAV: x.x, y.y]."
        )
        
        # 4. Start the Web UI (Hosted on port 8050)
        self.web_ui = WebServer(self.model)
        
        # Override the Web UI's chat hook to intercept user requests
        self.web_ui.on_message = self.handle_chat_request
        threading.Thread(target=self.web_ui.start).start()
        
    def image_cb(self, msg):
        # Silently keep the freshest frame in memory (~30FPS)
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def map_cb(self, msg):
        # Convert 1D occupancy grid to a 2D top-down image
        width, height = msg.info.width, msg.info.height
        if width == 0 or height == 0: return
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[data == -1] = [200, 200, 200]  # Unknown = gray
        img[data == 0] = [255, 255, 255]   # Free = white
        img[data > 0] = [0, 0, 0]          # Obstacle = black
        
        # ROS origin is bottom-left, CV2 images are origin top-left usually
        self.latest_nvblox_map_image = np.flipud(img)

    def handle_chat_request(self, user_text):
        # Triggered ONLY when you type/say something in the web chat
        print(f"User requested: {user_text}")
        
        if self.latest_image is None:
            return "Camera not ready yet."
            
        # 1. Provide the dormant images to the VLM
        images_to_send = [self.latest_image]
        context = user_text
        
        if self.latest_nvblox_map_image is not None:
            images_to_send.append(self.latest_nvblox_map_image)
            context += "\n[System: Note, I am supplying you two images. The first is my camera view. The second is an nvblox map.]"
            
        vlm_response = self.model.generate(context, image=images_to_send)
        
        # 2. Parse the output for our secret [NAV: x, y] command
        match = re.search(r'\[NAV:\s*([\d\.\-]+),\s*([\d\.\-]+)\]', vlm_response)
        if match:
            x, y = float(match.group(1)), float(match.group(2))
            
            # 3. Publish to ROS 2 Nav2
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = x
            goal.pose.position.y = y
            self.nav_pub.publish(goal)
            
            # Clean the tag out of the response so the user doesn't see it in chat
            vlm_response = re.sub(r'\[NAV:.*?\]', '', vlm_response)
            
        return vlm_response

def main(args=None):
    rclpy.init(args=args)
    node = EventGatedBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
