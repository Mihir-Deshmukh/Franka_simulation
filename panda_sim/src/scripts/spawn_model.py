#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import random
import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler

def spawn_model(model_name, model_path, model_pose):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf(model_name, open(model_path, 'r').read(), "/", model_pose, "world")
        rospy.loginfo(f'Spawned {model_name} successfully at x={model_pose.position.x}, y={model_pose.position.y}, z={model_pose.position.z}!')
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    rospy.sleep(0.1)

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model(model_name)
        rospy.loginfo(f'Deleted {model_name} successfully!')
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def generate_random_pose(x_min, x_max, y_min, y_max, z):
    pose = Pose()
    pose.position.x = random.uniform(x_min, x_max)
    pose.position.y = random.uniform(y_min, y_max)
    pose.position.z = z
    return pose

def generate_random_pose_new(x_min, x_max, y_min, y_max, z, model_name):
    pose = Pose()
    pose.position.x = random.uniform(x_min, x_max)
    pose.position.y = random.uniform(y_min, y_max)
    pose.position.z = z
    
    if model_name == "chips_can":
        pose.position.z = z + 0.1
        
        pos_choice = random.uniform(0, 1)
        if pos_choice < 0.5: #bias so that the can placed 50% times towards the wall
            pose.position.y = random.uniform(0.15, y_max- 0.04)
        else:
            pose.position.y = random.uniform(y_min+0.07, y_max- 0.07)
        
        # Remove this after bias    
        pose.position.y = random.uniform(0.32, 0.37)   
        
        pose.position.x = random.uniform(x_min-0.1, x_max - 0.1)
        
        roll = np.pi / 2
        pitch = 0
        
        # Generate a random value to determine the yaw angle
        yaw_choice = random.uniform(0, 1)
        if yaw_choice < 0.6:  # 60% chance of yaw being 90 degrees
            yaw = np.pi/2
        elif yaw_choice < 0.75:  # 15% chance of yaw being 0 degrees
            yaw = 0
        else:  # 20% chance of random yaw between 0 and 180 degrees
            yaw = random.uniform(0, np.pi)
        
        yaw = np.pi/2
        
        if yaw_choice < 0.5:
            yaw = 70 * np.pi / 180
        else:
            yaw = 100 * np.pi / 180
            
        # Convert Euler angles to quaternion using tf.transformations
        quat = quaternion_from_euler(roll, pitch, yaw)
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
    elif model_name == "coffee_mug":
        roll = 0
        pitch = 0
        # Generate a random value to determine the yaw angle
        yaw = random.uniform(0, 2*np.pi)
        
        quat = quaternion_from_euler(roll, pitch, yaw)
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
            
    return pose

def save_depth_image(image_msg, save_path):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="32FC1")
    # Normalize the depth values to the 0-255 range for saving as an 8-bit image
    print(np.min(cv_image))
    print(np.max(cv_image))
    cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
    cv_image = cv_image.astype(np.uint8)
    cv_image = 255 - cv_image
    
    cv2.imwrite(save_path, cv_image)
    
def spawn_and_capture_objects(model_paths, num_samples, num_objects):
    # max attempts to find poses
    max_attempts_total = 200
    count = 0
    
    while count < num_samples:
        model_names = [f"object_{count}_{j}" for j in range(1, num_objects + 1)]
        # model_paths_to_spawn = random.sample(model_paths, num_objects)
        
        model_names_spawn = ["chips_can"] #remove this
        
        filtered_keys = [key for key in model_paths.keys() if key != "chips_can"]
        
        temp = random.sample(filtered_keys, num_objects-1) #change to num_objects
        # Assuming model_names_spawn already contains some items
        model_names_spawn.extend(temp)
        
        # model_names_spawn = random.sample(model_paths.keys(), num_objects) #change to num_objects

        model_paths_to_spawn = [model_paths[key] for key in model_names_spawn]
        print(model_names_spawn)
        poses = []
        for k in range(num_objects):
            attempts = 0
            while attempts < max_attempts_total:
                # rospy.loginfo("Finding suitable poses:")
                pose = generate_random_pose_new(x_min, x_max, y_min, y_max, fixed_z, model_names_spawn[k])
                if all(((pose.position.x - p.position.x) ** 2 + (pose.position.y - p.position.y) ** 2) ** 0.5 >= 0.3 for p in poses):
                    poses.append(pose)
                    break
                if attempts >= max_attempts_total:
                    rospy.loginfo(f"Warning: Maximum attempts reached for object {model_names_spawn[k]}. Unable to find a suitable pose.")
             
                attempts += 1        
                
        if len(poses) >= num_objects:
            count += 1
            
            for j in range(num_objects):
                spawn_model(model_names[j], model_paths_to_spawn[j], poses[j])

            # Capture and save the depth image
            # wait_for_depth_image()
            rospy.sleep(0.2)  # Wait for a moment to ensure the depth image is available
            if depth_image_msg is not None:
                image_filename = os.path.join(save_dir, f"{count+119}_depth_image_{'_'.join(model_names_spawn)}.png")
                save_depth_image(depth_image_msg, image_filename)

            for model_name in model_names:
                delete_model(model_name)  # Delete the objects immediately after spawning
        else:
            rospy.loginfo("Error: Unable to find suitable poses for at least three objects.")
        
        
   
def spawn_and_capture(model_path, object_name_prefix, num_samples):
    for i in range(num_samples):
        model_name = f"{object_name_prefix}_{i}"
        model_pose = generate_random_pose_new(x_min, x_max, y_min, y_max, fixed_z, object_name_prefix)
        spawn_model(model_name, model_path, model_pose)

        # Capture and save the depth image
        # wait_for_depth_image()
        rospy.sleep(0.2)  # Wait for a moment to ensure the depth image is available
        if depth_image_msg is not None:
            image_filename = os.path.join(save_dir, f"depth_image_{object_name_prefix}_{i}.png")
            save_depth_image(depth_image_msg, image_filename)

        delete_model(model_name)  # Delete the object immediately after spawning

depth_image_msg = None
def depth_image_callback(msg):
    global depth_image_msg
    depth_image_msg = msg

# def wait_for_depth_image():
#     # Wait for a depth image to be received
#     while depth_image_msg is None:
#         rospy.sleep(0.2)

if __name__ == '__main__':
    rospy.init_node('spawn_models_node')
    
    # Define the file paths for the "bowl" and "coke_can" models
    plate_path = '/home/mihir/.gazebo/models/029_plate_textured/model.sdf'
    cricket_ball_path = '/home/mihir/.gazebo/models/Cricket_ball/model.sdf'
    chips_can_path = '/home/mihir/.gazebo/models/chips_can/model.sdf'
    coffee_mug_path = '/home/mihir/.gazebo/models/025_mug_textured/model.sdf'
    
    # Define the range and fixed z position
    x_min, x_max = 0.95, 1.4
    y_min, y_max = -0.3, 0.3
    fixed_z = 1.03

    # Create a directory to save depth images
    # save_dir = '/home/mihir/Projects/DataSet/Individual/'
    save_dir = '/home/mihir/Projects/DataSet/Two_objects/'
    # save_dir = '/home/mihir/Projects/DataSet/Three_objects/'
    
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Create a depth image subscriber
    depth_image_subscriber = rospy.Subscriber('/depth_camera/depth/image_raw', Image, depth_image_callback)

    # Generate 10 image samples for each of the four objects
    # num_samples = 6
    # spawn_and_capture(cricket_ball_path, 'cricket_ball', num_samples)
    # spawn_and_capture(chips_can_path, 'chips_can', num_samples)
    # spawn_and_capture(coffee_mug_path, 'coffee_mug', num_samples)
    # spawn_and_capture(plate_path, 'plate', num_samples)
    
    
    model_paths = [plate_path, cricket_ball_path, chips_can_path, coffee_mug_path]
    # object_names = ["plate", "ball", "chips_can", "coffee_mug"]
    object_names = ["ball", "chips_can", "coffee_mug"]
    
    
    # object_model_paths = {
    # "plate": plate_path,
    # "ball": cricket_ball_path,
    # "chips_can": chips_can_path,
    # "coffee_mug": coffee_mug_path
    # }
    
    object_model_paths = {
    "plate": plate_path,
    "ball": cricket_ball_path,
    "chips_can": chips_can_path,
    "coffee_mug": coffee_mug_path
    }
    
    # Generate samples with two random objects at a time
    # num_samples_two_objects = 60
    # spawn_and_capture_objects(object_model_paths, num_samples_two_objects, num_objects=2)
    
    # # # Generate samples with three random objects at a time
    num_samples_three_objects = 30
    spawn_and_capture_objects(object_model_paths, num_samples_three_objects, num_objects=3)

    rospy.spin()