import threading
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import paho.mqtt.client as mqtt
import json
import torch
from PIL import Image as PILImage
from zoedepth.models.builder import build_model
from zoedepth.utils.config import get_config
from zoedepth.utils.misc import pil_to_batched_tensor

# Initialize ROS node
rospy.init_node('sensor_data_publisher')

# Publishers for IMU, Distance, Original and Processed Images
imu_publisher = rospy.Publisher('/imu_data', Imu, queue_size=10)
distance_publisher = rospy.Publisher('/distance_data', Float32, queue_size=10)
original_image_publisher = rospy.Publisher('/original_image', Image, queue_size=10)
processed_depth_publisher = rospy.Publisher('/processed_depth', Image, queue_size=10)
bridge = CvBridge()

# Initialize ZoeDepth Model
conf = get_config("zoedepth", "infer")
model_zoe_n = build_model(conf)
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
zoe = model_zoe_n.to(DEVICE)

# Frame processing rate control
frame_count = 0
process_every_n_frames = 5

def process_and_publish_depth_frame(frame):
    global frame_count

    # Convert to PIL Image and process with ZoeDepth
    image = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    depth_numpy = zoe.infer_pil(image)  # Assuming infer_pil is the correct method

    # Normalize and convert depth map to 8-bit
    if depth_numpy.max() != depth_numpy.min():
        depth_numpy_normalized = (depth_numpy - depth_numpy.min()) / (depth_numpy.max() - depth_numpy.min())
    depth_numpy_normalized = (depth_numpy_normalized * 255).astype(np.uint8)
    depth_display = cv2.applyColorMap(depth_numpy_normalized, cv2.COLORMAP_JET)

    # Display processed frame
    cv2.imshow('Processed Depth Frame', depth_display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown.")
        return

    # Publish processed depth image
    depth_message = bridge.cv2_to_imgmsg(depth_numpy_normalized, encoding="passthrough")
    processed_depth_publisher.publish(depth_message)

    frame_count += 1

def on_mqtt_message(client, userdata, message):
    global frame_count
    try:
        nparr = np.frombuffer(message.payload, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is not None and frame.size > 0:
            # Publish original frame
            original_image_publisher.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            cv2.imshow('Original Frame', frame)  # Display the original frame

            if frame_count % process_every_n_frames == 0:
                threading.Thread(target=process_and_publish_depth_frame, args=(frame,)).start()

            frame_count += 1
    except Exception as e:
        print("Error processing MQTT message:", e)

def start_mqtt_client():
    client = mqtt.Client()
    client.on_message = on_mqtt_message
    client.connect("test.mosquitto.org", 1883, 60)
    client.subscribe("openmv/image_stream")  # Update your image stream topic
    client.loop_forever()

# MQTT clients for IMU and Distance
def mqtt_to_ros_imu():
    def on_message(client, userdata, message):
        data = json.loads(message.payload.decode("utf-8"))
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = data["accel"]["x"]
        imu_msg.linear_acceleration.y = data["accel"]["y"]
        imu_msg.linear_acceleration.z = data["accel"]["z"]
        imu_msg.angular_velocity.x = data["gyro"]["x"]
        imu_msg.angular_velocity.y = data["gyro"]["y"]
        imu_msg.angular_velocity.z = data["gyro"]["z"]
        imu_msg.header.stamp = rospy.Time.now()
        ros_publisher.publish(imu_msg)

    ros_publisher = rospy.Publisher('/imu_data', Imu, queue_size=10)
    client = mqtt.Client()
    client.connect("test.mosquitto.org", 1883, 60)
    client.subscribe("openmv/imu")
    client.on_message = on_message

    client.loop_start()
    rospy.spin()
    client.loop_stop()

def mqtt_to_ros_distance():
    def on_message(client, userdata, message):
        data = json.loads(message.payload.decode("utf-8"))
        # Extract the distance value and ensure it's a float
        distance_value = float(data['distance']) if 'distance' in data else 0.0
        distance_msg = Float32()
        distance_msg.data = distance_value
        distance_publisher.publish(distance_msg)

    distance_publisher = rospy.Publisher('/distance_data', Float32, queue_size=10)
    client = mqtt.Client()
    client.connect("test.mosquitto.org", 1883, 60)
    client.subscribe("openmv/distance")
    client.on_message = on_message

    client.loop_start()
    rospy.spin()
    client.loop_stop()

# Thread setup and execution
thread_mqtt_video = threading.Thread(target=start_mqtt_client)
thread_mqtt_ros_imu = threading.Thread(target=mqtt_to_ros_imu)
thread_mqtt_ros_distance = threading.Thread(target=mqtt_to_ros_distance)

thread_mqtt_video.start()
thread_mqtt_ros_imu.start()
thread_mqtt_ros_distance.start()

thread_mqtt_video.join()
thread_mqtt_ros_imu.join()
thread_mqtt_ros_distance.join()
