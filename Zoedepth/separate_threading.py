import threading
import multiprocessing
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
from zoedepth.models.builder import build_model
from zoedepth.utils.config import get_config
import torch
import numpy as np
from zoedepth.utils.misc import pil_to_batched_tensor


# Initialize ROS node
rospy.init_node('depth_publisher')
pub = rospy.Publisher('processed_depth', Image, queue_size=10)
bridge = CvBridge()

# Initialize ZoeDepth Model
#torch.hub.help("intel-isl/MiDaS", "DPT_BEiT_L_512", force_reload=True) 
#torch.hub.help("intel-isl/MiDaS", "DPT_Next_ViT_L_384", force_reload=True) 
conf = get_config("zoedepth", "infer")
model_zoe_n = build_model(conf)
repo = "isl-org/ZoeDepth"
# Zoe_N
model_zoe_n = torch.hub.load(".", "ZoeD_N", source="local", pretrained=True)
#model_zoe_n = torch.hub.load(repo, "ZoeD_N", pretrained=True)
DEVICE = "cuda" 
print(DEVICE)
zoe = model_zoe_n.to(DEVICE)
global depth_numpy

# Open the RTSP stream
#cap = cv2.VideoCapture('rtsp://172.20.10.5:554')
cap = cv2.VideoCapture('rtsp://192.168.137.132:554')

depth_map = None
depth_map_lock = threading.Lock()

def process_frame(frame):
    global depth_map

    # Convert to PIL Image
    image = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # Process the frame with ZoeDepth
    depth_numpy = zoe.infer_pil(image)  # Assuming infer_pil is the correct method
    depth_message = bridge.cv2_to_imgmsg(depth_numpy, encoding="passthrough")
    pub.publish(depth_message)

    # Normalize and convert depth map to 8-bit
    if depth_numpy.max() != depth_numpy.min():  # Avoid division by zero
        depth_numpy = (depth_numpy - depth_numpy.min()) / (depth_numpy.max() - depth_numpy.min())
    depth_numpy = (depth_numpy * 255).astype(np.uint8)

    # Update the shared depth_map
    with depth_map_lock:
        depth_map = depth_numpy

    # Convert processed depth image to ROS message and publish


def stream_video():
    global depth_map
    frame_count = 0

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # Display the original frame
        cv2.imshow('Frame', frame)

        # Check and display the depth map if available
        with depth_map_lock:
            if depth_map is not None:
                cv2.imshow('Depth', depth_map)
                depth_map = None  # Reset depth map after displaying

        # Process every 5th frame
        if frame_count % 10 == 0:
        #if 1:
            threading.Thread(target=process_frame, args=(frame,)).start()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_count += 1

    cap.release()
    cv2.destroyAllWindows()

# Run the streaming in the main thread
stream_video()