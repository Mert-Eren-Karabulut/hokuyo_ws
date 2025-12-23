#ifndef HOKUYO_GO_POINT_CLOUD_SEGMENTER_H
#define HOKUYO_GO_POINT_CLOUD_SEGMENTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include <vector>
#include <map>
#include <cmath>

namespace hokuyo_go
{

/**
 * @brief LCCP-based Point Cloud Segmenter
 * 
 * Implements segmentation using Locally Convex Connected Patches (LCCP):
 * 1. Optional RANSAC plane extraction (removes floor, walls, ceiling)
 * 2. Supervoxel over-segmentation (creates small coherent patches)
 * 3. LCCP merging based on convexity criterion
 * 
 * This approach works well for:
 * - Non-uniform density point clouds
 * - Moving scanner scenarios (view-independent)
 * - Connected/continuous indoor environments
 * 
 * Objects tend to be locally convex; boundaries between objects are concave.
 */
class PointCloudSegmenter
{
public:
    PointCloudSegmenter(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~PointCloudSegmenter();

private:
    // ROS interface
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    
    ros::Subscriber pointcloud_sub_;
    ros::Publisher segmented_cloud_pub_;
    ros::Publisher supervoxel_cloud_pub_;
    ros::Publisher planes_cloud_pub_;
    ros::Publisher cluster_markers_pub_;
    ros::Publisher adjacency_pub_;
    
    // Parameters
    // Plane segmentation (optional preprocessing)
    bool enable_plane_removal_;
    double plane_distance_threshold_;
    int plane_max_iterations_;
    int min_plane_points_;
    int max_planes_;
    
    // Supervoxel parameters
    float voxel_resolution_;        // Resolution of voxels (m)
    float seed_resolution_;         // Resolution of supervoxel seeds (m)
    float color_importance_;        // Weight for color in clustering
    float spatial_importance_;      // Weight for spatial distance
    float normal_importance_;       // Weight for surface normals
    
    // LCCP parameters
    float concavity_tolerance_threshold_;  // Angle threshold for concavity (degrees)
    float smoothness_threshold_;           // Smoothness constraint threshold
    int min_segment_size_;                 // Minimum points per segment
    bool use_extended_convexity_;          // Use extended convexity criterion
    bool use_sanity_criterion_;            // Use sanity check for merging
    
    // Clustering filter
    int min_cluster_size_;
    int max_cluster_size_;
    
    // Processing rate
    double processing_rate_;
    ros::Time last_process_time_;
    
    // Callbacks
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    // Processing pipeline
    void processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    
    // Step 1: Optional plane extraction using RANSAC
    void extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes_cloud,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_cloud);
    
    // Step 2: Supervoxel clustering
    void computeSupervoxels(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
                            std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                            pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_voxel_cloud);
    
    // Step 3: LCCP segmentation
    void performLCCP(std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
                     std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                     pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud);
    
    // Visualization
    void publishSupervoxelAdjacency(const std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
                                    const std::multimap<uint32_t, uint32_t>& supervoxel_adjacency);
    
    void publishClusterMarkers(pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud,
                               int num_labels);
    
    // Utility
    void getClusterColor(uint32_t label, uint8_t& r, uint8_t& g, uint8_t& b);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr labeledToRGB(pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud);
};

} // namespace hokuyo_go

#endif // HOKUYO_GO_POINT_CLOUD_SEGMENTER_H
