#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    ### TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    ### TODO: Voxel Grid Downsampling
    # Create voxel grid filter
    vox = cloud.make_voxel_grid_filter()
    # Select a leaf/voxel size - note that 1.0 is very large.
    # The leaf size is measured in meters. Therefore a size of 1 is a cubic meter.
    LEAF_SIZE = 0.01 # experiment with this
    # Set the voxel/leaf size. 
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
        
    ### TODO: PassThrough Filter
    # Create a passthrough filter object for the z-axis
    passthrough_z = cloud_filtered.make_passthrough_filter()
    # Assign axis and range of passthrough filter object.
    filter_axis = 'z'
    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = 0.735 # experiment with this
    axis_max = 1.1 # experiment with this
    passthrough_z.set_filter_limits(axis_min, axis_max)
    # Filter the downsampled point cloud with the passthrough object to get resultant cloud.
    cloud_filtered = passthrough_z.filter()

    # Create a passthrough filter for the y-axis to remove the front table edge
    passthrough_y = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough_y.set_filter_field_name(filter_axis)
    axis_min = -2.26
    axis_max = -1.35
    passthrough_y.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough_y.filter()

    ### TODO: RANSAC Plane Segmentation
    # Create segmentation model
    seg = cloud_filtered.make_segmenter()
    # Set the model to fit the objects to
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Set the max distance for a point to be considered to be fitting the model
    max_distance = 0.001 # experiement with this
    seg.set_distance_threshold(max_distance)

    ### TODO: Extract inliers and outliers
    # Call the segment function to obtain the set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    pcl_cloud_objects = cloud_filtered.extract(inliers, negative=True)
    pcl_cloud_table = cloud_filtered.extract(inliers, negative=False)

    ### TODO: Euclidean Clustering
    # Convert XYZRGB point cloud to XYZ as Euclidean Clustering cannot use colour information
    white_cloud = XYZRGB_to_XYZ(pcl_cloud_objects)
    # Construct the k-d tree
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as a minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.05) #experiment
    ec.set_MinClusterSize(200) #experiment
    ec.set_MaxClusterSize(2000) #experiment
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered indices
    cluster_indices = ec.Extract()
    # cluster_indices now contains a list of indices for each cluster (a list of lists)

    ### TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # This cloud will contain points for each of the segmented objects, with each set of points having a unique colour.
    # Assign a colour corresponding to each segmented object in the camera view
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud object containing all clusters, each with a unique colour
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    
    ### TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(pcl_cloud_objects)
    ros_cloud_table = pcl_to_ros(pcl_cloud_table)
    ros_cloud_clusters = pcl_to_ros(cluster_cloud)
    
    ### TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_cloud.publish(ros_cloud_clusters)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_cloud = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)
    
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
