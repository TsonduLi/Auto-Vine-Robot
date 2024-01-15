#!/usr/bin/env python3
import rospy
import math
import os
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import tf2_ros
#import tf_conversions
import geometry_msgs.msg
import re
import open3d as o3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

#import transforms3d

depth_array = None

def depth_array_to_pointcloud2(depth_array, segmentation_array,focal_length, frame_id="camera_link"):

    '''    if depth_array.shape != segmentation_array.shape:
        raise ValueError("Depth array and segmentation array must have the same shape")'''


    height, width = depth_array.shape
    max_depth = np.nanmax(depth_array)

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    
    #z / focal_length   
    point_cloud_data = []
    for i in range(height):
        for j in range(width):
            #if 1:
            if depth_array[i, j] != 0.0:
                #z = max_depth - depth_array[i, j]
                #z = 0.70785560 * (1/depth_array[i, j]) - 0.10008919 
                #z = 3.1155090 * (1/depth_array[i, j]) - 0.109684 
                z = (depth_array[i, j])/2.5
                x = (j - width / 2.0) * z / focal_length
                y = (i - height / 2.0) * z / focal_length
                point_cloud_data.append([x, y, z])
            
    pc2_msg = pc2.create_cloud(header, fields, point_cloud_data)
    return pc2_msg

def apply_statistical_outlier_removal_to_pointcloud2(cloud_msg, nb_neighbors=20, std_ratio=1.0):
    # Convert ROS PointCloud2 to Open3D point cloud
    cloud_points = list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")))
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(cloud_points)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             

    # Apply statistical outlier removal filter
    cloud_filtered, _ = o3d_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    # Convert filtered Open3D cloud back to ROS PointCloud2
    ros_cloud_filtered = PointCloud2()
    ros_cloud_filtered.header.stamp = rospy.Time.now()
    ros_cloud_filtered.header.frame_id = cloud_msg.header.frame_id
    ros_cloud_filtered.height = 1
    ros_cloud_filtered.width = len(cloud_filtered.points)

    ros_cloud_filtered.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
    ]

    ros_cloud_filtered.is_bigendian = False
    ros_cloud_filtered.point_step = 12
    ros_cloud_filtered.row_step = 12 * ros_cloud_filtered.width
    ros_cloud_filtered.is_dense = int(np.isfinite(np.asarray(cloud_filtered.points)).all())
    ros_cloud_filtered.data = np.asarray(cloud_filtered.points, dtype=np.float32).tobytes()
    return ros_cloud_filtered


def calculate_focal_length_pixels(image_width_pixels, sensor_width_mm, focal_length_mm):
    return (image_width_pixels * focal_length_mm) / sensor_width_mm

def broadcast_static_transform():
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "camera_link"

    # Setting translation to zero
    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = 0

    # Setting rotation to represent zero rotation
    angle_radians = math.radians(90)

    static_transformStamped.transform.rotation.x = -math.sin(angle_radians / 2)
    static_transformStamped.transform.rotation.y = 0
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = math.cos(angle_radians / 2)

    broadcaster.sendTransform(static_transformStamped)
'''def depth_image_callback(msg):
    global depth_array
    bridge = CvBridge()
    try:
        rospy.loginfo(f"Received image with encoding: {msg.encoding}")
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_array = np.array(cv_image, dtype=np.float32)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))'''


def depth_image_callback(msg):
    global depth_array
    try:
        # Manually convert ROS Image message to a numpy array
        if msg.encoding == '32FC1':
            dtype = np.dtype("float32")
            dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
            depth_array = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    except Exception as e:
        rospy.logerr("Error processing depth image: %s", e)

def process_depth_array_fast(depth_array, depth_threshold, gradient_threshold, dilation_iterations):
    # Convert depth_array to float if it's not already, for more precise computations
    depth_array = depth_array.astype(float)

    # Thresholding for small depth values using OpenCV
    _, small_depth = cv2.threshold(depth_array, depth_threshold, 1, cv2.THRESH_BINARY_INV)

    # Dilate the identified small depth areas using OpenCV
    dilated_small_depth = cv2.dilate(small_depth.astype(np.uint8), None, iterations=dilation_iterations)

    # Calculating the gradient magnitude
    gradient_magnitude = cv2.Laplacian(depth_array, cv2.CV_64F)
    _, large_gradient = cv2.threshold(gradient_magnitude, gradient_threshold, 1, cv2.THRESH_BINARY)

    # Combining the criteria
    main_body = cv2.bitwise_or(dilated_small_depth, large_gradient.astype(np.uint8))

    # Create a modified depth map
    modified_depth_map = np.where(main_body == 1, depth_array, 0.0)

    return modified_depth_map

if __name__ == "__main__":
    rospy.init_node('depth_to_pointcloud')
    pub = rospy.Publisher("/camera/depth/pointcloud", PointCloud2, queue_size=2)
    
    # Camera specifications for iPhone 11 Pro (main camera)
    image_width_pixels = 320
    sensor_width_mm = image_width_pixels*0.00175  # The sensor width in millimeters
    focal_length_mm = 2.2  # The focal length in millimeters (converted from 26mm equivalent)
    
    # Image specifications (replace with your actual image width)
    
    
    # Calculate the focal length in pixels
    focal_length_pixels = calculate_focal_length_pixels(image_width_pixels, sensor_width_mm, focal_length_mm)
    
    output_dir = '/home/yimeng/Downloads/dinov2-main/depth_value'
    os.makedirs(output_dir, exist_ok=True)

# Define the output file path
    output_file_path = os.path.join(output_dir, '/home/yimeng/ZoeDepth/pred_depth_nk.npy')
    # Generate or load your depth array here (replace with your actual depth array)
    #depth_array = np.load(output_file_path)# Example depth array

    output_dir_seg = '/home/yimeng/Downloads/dinov2-main/segment_result'
    os.makedirs(output_dir_seg, exist_ok=True)
    output_file_path_seg = os.path.join(output_dir_seg, 'segmentation_logits.npy')
    segmentation_array = np.load(output_file_path_seg)# Example depth array

    rospy.Subscriber("/processed_depth", Image, depth_image_callback)
    
    broadcast_static_transform()
    rate = rospy.Rate(10)
    log_counter = 0  # Counter to control the frequency of log messages
    log_frequency = 10  # Log every 10 iterations

    while not rospy.is_shutdown():
        if depth_array is not None:
            #print(depth_array.shape)
            modified_depth_map = process_depth_array_fast(depth_array, depth_threshold=3, gradient_threshold=10, dilation_iterations=1)
            pointcloud_msg = depth_array_to_pointcloud2(modified_depth_map, segmentation_array,focal_length_pixels)
            pointcloud_msg = apply_statistical_outlier_removal_to_pointcloud2(pointcloud_msg)
            pub.publish(pointcloud_msg)
        else:
            if log_counter % log_frequency == 0:
                rospy.loginfo("Waiting for depth data...")
            log_counter += 1
        rate.sleep()  # Publish at 10 Hz


