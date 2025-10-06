#!/usr/bin/env python

import rospy # type: ignore
from sensor_msgs.msg import PointCloud2 # type: ignore
import sensor_msgs.point_cloud2 as pc2 # type: ignore
from pcl import PointCloud, PointIndices # type: ignore
from pcl import segmentation, filters # type: ignore
import numpy as np


class DepthSegmentation:
    def __init__(self):
        rospy.init_node('depth_segmentation')

        # Parameters
        self.min_depth = rospy.get_param('~min_depth', 0.01)  # Minimum depth threshold
        self.max_depth = rospy.get_param('~max_depth', 5.5)  # Maximum depth threshold

        # ROS Subscribers and Publishers
        self.pc_sub = rospy.Subscriber('/output', PointCloud2, self.pc_callback)
        self.segmented_pc_pub = rospy.Publisher('/segmented_pointcloud', PointCloud2, queue_size=1)

        rospy.spin()

    def pc_callback(self, pc_msg):
        # Convert ROS PointCloud2 message to PCL PointCloud
        pcl_pc = PointCloud()
        pcl_pc.from_list(pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True))

        # Perform segmentation based on depth thresholds
        segmented_indices = self.segment_depth(pcl_pc)

        # Publish segmented PointCloud
        segmented_pc_msg = pc2.create_cloud_xyz32(pc_msg.header, pcl_pc.to_array())
        self.segmented_pc_pub.publish(segmented_pc_msg)

    def segment_depth(self, pcl_pc):
        # Apply depth thresholds to segment the point cloud
        seg = pcl_pc.make_segmenter()
        seg.set_filter_limits(self.min_depth, self.max_depth)
        indices = PointIndices()
        seg.segment(indices)

        return indices


if __name__ == '__main__':
    try:
        DepthSegmentation()
    except rospy.ROSInterruptException:
        pass
