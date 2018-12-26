#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(20)
    outlier_filter.set_std_dev_mul_thresh(0.1)
    cloud_filtered = outlier_filter.filter()
    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    # TODO: PassThrough Filter
    passthrough_y = cloud_filtered.make_passthrough_filter()
    passthrough_y.set_filter_field_name('y')
    passthrough_y.set_filter_limits(-0.4, 0.4)
    cloud_filtered = passthrough_y.filter()
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.61, 0.9)
    cloud_filtered = passthrough.filter()
    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
    # TODO: Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(3000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2], rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        # Compute the associated feature vector
        ros_cloud = pcl_to_ros(pcl_cluster)
        histogram_bins = 128
        chists = compute_color_histograms(ros_cloud, nbins=histogram_bins, using_hsv=True)
        normals = get_normals(ros_cloud)
        nhists = compute_normal_histograms(normals, nbins=histogram_bins)
        feature = np.concatenate((chists, nhists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cloud
        detected_objects.append(do)
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    # pcl_objects_pub.publish(ros_cloud_objects)
    # pcl_cluster_pub.publish(ros_cluster_cloud)
    # pcl_table_pub.publish(ros_cloud_table)
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    output_list = []
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')
    # TODO: Parse parameters into individual variables
    test_num = 1
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for object_params in object_list_param:
        object_name = object_params['name']
        object_group = object_params['group']
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        for object_i, object_val in enumerate(object_list):
            if object_name != object_val.label:
                continue
            ros_scene_num = Int32()
            # test_num = 1
            ros_scene_num.data = test_num
            ros_object_name = String()
            ros_object_name.data = object_name
            ros_arm_to_use = String()
            if object_group == 'green':
                ros_arm_to_use.data = 'right'
            else:
                ros_arm_to_use.data = 'left'
            points_array = ros_to_pcl(object_val.cloud).to_array()
            centroid_numpy = np.mean(points_array, axis=0)[:3]
            centroid = [np.asscalar(x) for x in centroid_numpy]
            ros_pick_pose = Pose()
            ros_pick_pose.position.x = centroid[0]
            ros_pick_pose.position.y = centroid[1]
            ros_pick_pose.position.z = centroid[2]
        # TODO: Create 'place_pose' for the object
            box_pos = [0, 0, 0]
            for box_params in dropbox_list_param:
                if box_params['group'] == object_group:
                    box_pos = box_params['position']
                    break
            ros_place_pos = Pose()
            ros_place_pos.position.x = box_pos[0]
            ros_place_pos.position.y = box_pos[1]
            ros_place_pos.position.z = box_pos[2]
        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            obj_yaml_dict = make_yaml_dict(ros_scene_num, ros_arm_to_use, ros_object_name, ros_pick_pose, ros_place_pos)
            output_list.append(obj_yaml_dict)
            print('processed %s' % ros_object_name.data)
        # Wait for 'pick_place_routine' service to come up
        # rospy.wait_for_service('pick_place_routine')

        # try:
            # pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            # resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            # print ("Response: ",resp.success)

        # except rospy.ServiceException, e:
            # print "Service call failed: %s"%e
            del object_list[object_i]
            break
    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_%i.yaml' % test_num, output_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('perception', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('pcl_cluster', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
