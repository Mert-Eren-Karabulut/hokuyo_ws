#include "hokuyo_go/modular_scanner/point_cloud_segmenter.h"

#include <pcl/filters/voxel_grid.h>
#include <set>

namespace hokuyo_go
{

PointCloudSegmenter::PointCloudSegmenter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    // Load parameters
    // Plane segmentation parameters (optional)
    pnh_.param("enable_plane_removal", enable_plane_removal_, true);
    pnh_.param("plane_distance_threshold", plane_distance_threshold_, 0.03);
    pnh_.param("plane_max_iterations", plane_max_iterations_, 100);
    pnh_.param("min_plane_points", min_plane_points_, 300);
    pnh_.param("max_planes", max_planes_, 2);
    
    // Supervoxel parameters
    pnh_.param("voxel_resolution", voxel_resolution_, 0.02f);      // 2cm voxels
    pnh_.param("seed_resolution", seed_resolution_, 0.08f);        // 8cm seeds
    pnh_.param("color_importance", color_importance_, 0.0f);       // No color (our points are colored by level)
    pnh_.param("spatial_importance", spatial_importance_, 1.0f);   // Full spatial weight
    pnh_.param("normal_importance", normal_importance_, 4.0f);     // High normal weight
    
    // LCCP parameters
    pnh_.param("concavity_tolerance_threshold", concavity_tolerance_threshold_, 10.0f);  // 10 degrees
    pnh_.param("smoothness_threshold", smoothness_threshold_, 0.1f);
    pnh_.param("min_segment_size", min_segment_size_, 3);          // Min supervoxels per segment
    pnh_.param("use_extended_convexity", use_extended_convexity_, true);
    pnh_.param("use_sanity_criterion", use_sanity_criterion_, true);
    
    // Cluster size filter
    pnh_.param("min_cluster_size", min_cluster_size_, 20);
    pnh_.param("max_cluster_size", max_cluster_size_, 100000);
    
    // Processing rate
    pnh_.param("processing_rate", processing_rate_, 1.0);  // 1 Hz default (LCCP is expensive)
    
    last_process_time_ = ros::Time::now();
    
    // Setup subscribers and publishers
    pointcloud_sub_ = nh_.subscribe("output", 1, &PointCloudSegmenter::pointCloudCallback, this);
    
    segmented_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    supervoxel_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("supervoxel_cloud", 1);
    planes_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("planes_cloud", 1);
    cluster_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);
    adjacency_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("supervoxel_adjacency", 1);
    
    ROS_INFO("=== LCCP Point Cloud Segmenter Initialized ===");
    ROS_INFO("Subscribing to: %s", pointcloud_sub_.getTopic().c_str());
    ROS_INFO("Publishing segmented cloud to: segmented_cloud");
    ROS_INFO("Publishing supervoxels to: supervoxel_cloud");
    ROS_INFO("Processing rate: %.1f Hz", processing_rate_);
    ROS_INFO("Plane removal: %s (threshold=%.3fm, max_planes=%d)", 
             enable_plane_removal_ ? "enabled" : "disabled",
             plane_distance_threshold_, max_planes_);
    ROS_INFO("Supervoxel: voxel_res=%.3fm, seed_res=%.3fm", voxel_resolution_, seed_resolution_);
    ROS_INFO("Supervoxel weights: color=%.1f, spatial=%.1f, normal=%.1f",
             color_importance_, spatial_importance_, normal_importance_);
    ROS_INFO("LCCP: concavity=%.1fdeg, smoothness=%.2f, min_segment=%d",
             concavity_tolerance_threshold_, smoothness_threshold_, min_segment_size_);
    ROS_INFO("Extended convexity: %s, Sanity criterion: %s",
             use_extended_convexity_ ? "yes" : "no",
             use_sanity_criterion_ ? "yes" : "no");
}

PointCloudSegmenter::~PointCloudSegmenter() {}

void PointCloudSegmenter::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Rate limiting
    ros::Time now = ros::Time::now();
    double dt = (now - last_process_time_).toSec();
    if (dt < 1.0 / processing_rate_)
    {
        return;
    }
    last_process_time_ = now;
    
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty())
    {
        ROS_WARN_THROTTLE(5.0, "Received empty point cloud");
        return;
    }
    
    ROS_DEBUG("Received cloud with %zu points", cloud->size());
    
    processPointCloud(cloud);
}

void PointCloudSegmenter::processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    ros::Time start_time = ros::Time::now();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Step 1: Optional plane extraction
    if (enable_plane_removal_)
    {
        extractPlanes(cloud, planes_cloud, working_cloud);
        ROS_DEBUG("After plane removal: %zu plane points, %zu remaining", 
                  planes_cloud->size(), working_cloud->size());
        
        // Publish planes
        if (!planes_cloud->empty())
        {
            sensor_msgs::PointCloud2 planes_msg;
            pcl::toROSMsg(*planes_cloud, planes_msg);
            planes_msg.header.frame_id = "map";
            planes_msg.header.stamp = ros::Time::now();
            planes_cloud_pub_.publish(planes_msg);
        }
    }
    else
    {
        *working_cloud = *cloud;
    }
    
    if (working_cloud->size() < 100)
    {
        ROS_DEBUG_THROTTLE(2.0, "Not enough points for LCCP segmentation (%zu)", working_cloud->size());
        return;
    }
    
    // Step 2: Supervoxel clustering
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters;
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    
    computeSupervoxels(working_cloud, supervoxel_clusters, supervoxel_adjacency, labeled_voxel_cloud);
    
    if (supervoxel_clusters.empty())
    {
        ROS_WARN_THROTTLE(2.0, "Supervoxel clustering produced no clusters");
        return;
    }
    
    ROS_DEBUG("Created %zu supervoxels", supervoxel_clusters.size());
    
    // Publish supervoxel adjacency for visualization
    publishSupervoxelAdjacency(supervoxel_clusters, supervoxel_adjacency);
    
    // Step 3: LCCP segmentation (relabels the labeled_voxel_cloud in place)
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud = labeled_voxel_cloud;
    
    performLCCP(supervoxel_clusters, supervoxel_adjacency, labeled_cloud);
    
    if (labeled_cloud->empty())
    {
        ROS_WARN_THROTTLE(2.0, "LCCP produced no labeled points");
        return;
    }
    
    // Count unique labels
    std::set<uint32_t> unique_labels;
    for (const auto& pt : *labeled_cloud)
    {
        if (pt.label > 0)  // 0 is usually unlabeled
        {
            unique_labels.insert(pt.label);
        }
    }
    
    int num_segments = static_cast<int>(unique_labels.size());
    
    double elapsed_ms = (ros::Time::now() - start_time).toSec() * 1000.0;
    ROS_INFO("LCCP segmentation: %d segments from %zu points (%.1f ms)",
             num_segments, working_cloud->size(), elapsed_ms);
    
    // Convert to RGB and publish
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_rgb = labeledToRGB(labeled_cloud);
    
    if (!segmented_rgb->empty())
    {
        sensor_msgs::PointCloud2 seg_msg;
        pcl::toROSMsg(*segmented_rgb, seg_msg);
        seg_msg.header.frame_id = "map";
        seg_msg.header.stamp = ros::Time::now();
        segmented_cloud_pub_.publish(seg_msg);
    }
    
    // Publish cluster markers
    publishClusterMarkers(labeled_cloud, num_segments);
}

void PointCloudSegmenter::extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes_cloud,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_cloud)
{
    *remaining_cloud = *cloud;
    
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(plane_max_iterations_);
    seg.setDistanceThreshold(plane_distance_threshold_);
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    
    int planes_extracted = 0;
    
    while (remaining_cloud->size() > static_cast<size_t>(min_plane_points_) && 
           planes_extracted < max_planes_)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < static_cast<size_t>(min_plane_points_))
        {
            break;
        }
        
        // Extract plane points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_points);
        
        // Color plane points gray
        for (auto& pt : *plane_points)
        {
            pt.r = 128;
            pt.g = 128;
            pt.b = 128;
        }
        
        *planes_cloud += *plane_points;
        
        // Remove plane points from remaining
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*temp);
        remaining_cloud.swap(temp);
        
        planes_extracted++;
        
        ROS_DEBUG("Extracted plane %d with %zu points", planes_extracted, plane_points->size());
    }
}

void PointCloudSegmenter::computeSupervoxels(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
    std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
    pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_voxel_cloud)
{
    // Create supervoxel clustering object
    pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resolution_, seed_resolution_);
    
    super.setInputCloud(cloud);
    super.setColorImportance(color_importance_);
    super.setSpatialImportance(spatial_importance_);
    super.setNormalImportance(normal_importance_);
    
    // Perform clustering
    super.extract(supervoxel_clusters);
    
    if (supervoxel_clusters.empty())
    {
        ROS_WARN("Supervoxel extraction failed");
        return;
    }
    
    // Get adjacency map
    super.getSupervoxelAdjacency(supervoxel_adjacency);
    
    // Get labeled voxel cloud (needed for LCCP relabeling)
    labeled_voxel_cloud = super.getLabeledVoxelCloud();
    
    // Publish supervoxel centroids for visualization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr supervoxel_centroids(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for (const auto& sv_pair : supervoxel_clusters)
    {
        pcl::PointXYZRGB pt;
        pt.x = sv_pair.second->centroid_.x;
        pt.y = sv_pair.second->centroid_.y;
        pt.z = sv_pair.second->centroid_.z;
        
        // Color by label
        uint8_t r, g, b;
        getClusterColor(sv_pair.first, r, g, b);
        pt.r = r;
        pt.g = g;
        pt.b = b;
        
        supervoxel_centroids->push_back(pt);
    }
    
    sensor_msgs::PointCloud2 sv_msg;
    pcl::toROSMsg(*supervoxel_centroids, sv_msg);
    sv_msg.header.frame_id = "map";
    sv_msg.header.stamp = ros::Time::now();
    supervoxel_cloud_pub_.publish(sv_msg);
}

void PointCloudSegmenter::performLCCP(
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
    std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
    pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud)
{
    // Create LCCP segmentation object
    pcl::LCCPSegmentation<pcl::PointXYZRGB> lccp;
    
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold_);
    lccp.setSmoothnessCheck(true, voxel_resolution_, seed_resolution_, smoothness_threshold_);
    lccp.setMinSegmentSize(min_segment_size_);
    lccp.setKFactor(use_extended_convexity_ ? 1 : 0);
    
    if (use_sanity_criterion_)
    {
        lccp.setSanityCheck(true);
    }
    
    // Set input and perform segmentation
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.segment();
    
    // Relabel the voxel cloud with segment labels (modifies in place)
    lccp.relabelCloud(*labeled_cloud);
}

void PointCloudSegmenter::publishSupervoxelAdjacency(
    const std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
    const std::multimap<uint32_t, uint32_t>& supervoxel_adjacency)
{
    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "supervoxel_adjacency";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.005;  // Line width
    line_marker.color.r = 0.5;
    line_marker.color.g = 0.5;
    line_marker.color.b = 0.5;
    line_marker.color.a = 0.5;
    line_marker.lifetime = ros::Duration(1.0 / processing_rate_ * 2);
    
    for (const auto& adj_pair : supervoxel_adjacency)
    {
        auto it1 = supervoxel_clusters.find(adj_pair.first);
        auto it2 = supervoxel_clusters.find(adj_pair.second);
        
        if (it1 != supervoxel_clusters.end() && it2 != supervoxel_clusters.end())
        {
            geometry_msgs::Point p1, p2;
            p1.x = it1->second->centroid_.x;
            p1.y = it1->second->centroid_.y;
            p1.z = it1->second->centroid_.z;
            p2.x = it2->second->centroid_.x;
            p2.y = it2->second->centroid_.y;
            p2.z = it2->second->centroid_.z;
            
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
        }
    }
    
    marker_array.markers.push_back(line_marker);
    adjacency_pub_.publish(marker_array);
}

void PointCloudSegmenter::publishClusterMarkers(
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud,
    int num_labels)
{
    // Compute statistics for each label
    std::map<uint32_t, std::vector<pcl::PointXYZL>> label_points;
    
    for (const auto& pt : *labeled_cloud)
    {
        if (pt.label > 0)
        {
            label_points[pt.label].push_back(pt);
        }
    }
    
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    
    for (const auto& lp : label_points)
    {
        uint32_t label = lp.first;
        const auto& points = lp.second;
        
        // Filter by size
        if (static_cast<int>(points.size()) < min_cluster_size_ ||
            static_cast<int>(points.size()) > max_cluster_size_)
        {
            continue;
        }
        
        // Compute centroid and bounding box
        Eigen::Vector3f min_pt(std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max());
        Eigen::Vector3f max_pt(std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest());
        Eigen::Vector3f centroid(0, 0, 0);
        
        for (const auto& pt : points)
        {
            centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
            min_pt[0] = std::min(min_pt[0], pt.x);
            min_pt[1] = std::min(min_pt[1], pt.y);
            min_pt[2] = std::min(min_pt[2], pt.z);
            max_pt[0] = std::max(max_pt[0], pt.x);
            max_pt[1] = std::max(max_pt[1], pt.y);
            max_pt[2] = std::max(max_pt[2], pt.z);
        }
        centroid /= static_cast<float>(points.size());
        
        uint8_t r, g, b;
        getClusterColor(label, r, g, b);
        
        // Bounding box marker
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = "map";
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "cluster_bbox";
        bbox_marker.id = marker_id++;
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        
        bbox_marker.pose.position.x = (min_pt[0] + max_pt[0]) / 2.0;
        bbox_marker.pose.position.y = (min_pt[1] + max_pt[1]) / 2.0;
        bbox_marker.pose.position.z = (min_pt[2] + max_pt[2]) / 2.0;
        bbox_marker.pose.orientation.w = 1.0;
        
        bbox_marker.scale.x = std::max(0.05f, max_pt[0] - min_pt[0]);
        bbox_marker.scale.y = std::max(0.05f, max_pt[1] - min_pt[1]);
        bbox_marker.scale.z = std::max(0.05f, max_pt[2] - min_pt[2]);
        
        bbox_marker.color.r = r / 255.0f;
        bbox_marker.color.g = g / 255.0f;
        bbox_marker.color.b = b / 255.0f;
        bbox_marker.color.a = 0.25;
        
        bbox_marker.lifetime = ros::Duration(1.0 / processing_rate_ * 2);
        
        marker_array.markers.push_back(bbox_marker);
        
        // Text label
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "cluster_label";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = centroid[0];
        text_marker.pose.position.y = centroid[1];
        text_marker.pose.position.z = max_pt[2] + 0.1;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.08;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        char label_text[64];
        snprintf(label_text, sizeof(label_text), "S%u (%zu)", label, points.size());
        text_marker.text = label_text;
        
        text_marker.lifetime = ros::Duration(1.0 / processing_rate_ * 2);
        
        marker_array.markers.push_back(text_marker);
    }
    
    cluster_markers_pub_.publish(marker_array);
}

void PointCloudSegmenter::getClusterColor(uint32_t label, uint8_t& r, uint8_t& g, uint8_t& b)
{
    // Generate distinct colors using golden ratio
    const float golden_ratio = 0.618033988749895f;
    float hue = std::fmod(label * golden_ratio, 1.0f);
    
    // HSV to RGB (saturation=0.85, value=0.95)
    float s = 0.85f;
    float v = 0.95f;
    
    int hi = static_cast<int>(hue * 6.0f);
    float f = hue * 6.0f - hi;
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);
    
    float rf, gf, bf;
    switch (hi % 6)
    {
        case 0: rf = v; gf = t; bf = p; break;
        case 1: rf = q; gf = v; bf = p; break;
        case 2: rf = p; gf = v; bf = t; break;
        case 3: rf = p; gf = q; bf = v; break;
        case 4: rf = t; gf = p; bf = v; break;
        default: rf = v; gf = p; bf = q; break;
    }
    
    r = static_cast<uint8_t>(rf * 255);
    g = static_cast<uint8_t>(gf * 255);
    b = static_cast<uint8_t>(bf * 255);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudSegmenter::labeledToRGB(
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Count points per label for filtering
    std::map<uint32_t, int> label_counts;
    for (const auto& pt : *labeled_cloud)
    {
        if (pt.label > 0)
        {
            label_counts[pt.label]++;
        }
    }
    
    for (const auto& pt : *labeled_cloud)
    {
        // Skip unlabeled points
        if (pt.label == 0)
            continue;
        
        // Skip small/large clusters
        int count = label_counts[pt.label];
        if (count < min_cluster_size_ || count > max_cluster_size_)
            continue;
        
        pcl::PointXYZRGB rgb_pt;
        rgb_pt.x = pt.x;
        rgb_pt.y = pt.y;
        rgb_pt.z = pt.z;
        
        uint8_t r, g, b;
        getClusterColor(pt.label, r, g, b);
        rgb_pt.r = r;
        rgb_pt.g = g;
        rgb_pt.b = b;
        
        rgb_cloud->push_back(rgb_pt);
    }
    
    return rgb_cloud;
}

} // namespace hokuyo_go
