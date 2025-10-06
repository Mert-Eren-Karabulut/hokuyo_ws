#include "segmentation.h"

#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/transforms.h>

#include "zed_params/zed2_parameters.h"
ZED2Parameters &ZED2Params = ZED2Parameters::getInstance();

// Helper functions inside a anonymous namespace
namespace
{

    // Function to calculate the distance between a point and a plane defined by coefficients
    float pointToPlaneDistance(const pcl::PointXYZRGB &point, const pcl::ModelCoefficients::Ptr &coefficients)
    {
        // Calculate the distance from the point to the plane using the plane equation
        return std::abs(coefficients->values[0] * point.x +
                        coefficients->values[1] * point.y +
                        coefficients->values[2] * point.z +
                        coefficients->values[3]) /
               std::sqrt(coefficients->values[0] * coefficients->values[0] +
                         coefficients->values[1] * coefficients->values[1] +
                         coefficients->values[2] * coefficients->values[2]);
    }

    // Function to find indices of points closer to a specified model than a threshold
    pcl::PointIndices::Ptr findPointsCloserToModel(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgbdCloud_,
        const pcl::ModelCoefficients::Ptr &model_coefficients,
        const float threshold)
    {

        pcl::PointIndices::Ptr indices(new pcl::PointIndices);

        // Iterate through each point in the point rgbdCloud_
        for (size_t i = 0; i < rgbdCloud_->size(); ++i)
        {
            const pcl::PointXYZRGB &point = rgbdCloud_->points[i];

            // Calculate the distance from the point to the specified model
            float distance = pointToPlaneDistance(point, model_coefficients);

            // Check if the absolute distance is less than the threshold
            if (std::abs(distance) < threshold)
            {
                indices->indices.push_back(static_cast<int>(i)); // Store the index of the point
            }
        }

        return indices;
    }

    bool compareZ(const pcl::PointXYZRGB &point1, const pcl::PointXYZRGB &point2)
    {
        return point1.z < point2.z;
    }

    void sortPointCloudByZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        std::sort(cloud->points.begin(), cloud->points.end(), compareZ);
    }
} // namespace

Segmentation::Segmentation(const SegmentationParams &params) : params_(params)
{
    rgbdCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    foreground_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    obstacles_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    depthSub_ = nh_.subscribe(params_.inputTopic, 1, &Segmentation::pointCloudCallback, this);
    // Create a publisher for the new PointCloud2 topic
    foregroundPub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.outputNamespace + "/foreground", 1);
    obstaclesPub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.outputNamespace + "/obstacles", 1);
    coloredCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.outputNamespace + "/segments", 1);

    panLimit_ = int(panEpsilon_ / (ZED2Params.rgb_fov_x / ZED2Params.width));
    tiltLimit_ = int(tiltEpsilon_ / (ZED2Params.rgb_fov_y / ZED2Params.height));

    numEstLPR_ = params_.numEstLPR;
    seedsThreshold_ = params_.seedsThreshold;
    distanceThreshold_ = params_.distanceThreshold;
    plane_distance_threshold_ = params_.plane_distance_threshold;
    min_points_required_for_plane_ = params_.min_points_required_for_plane;
    vertical_threshold_ = params_.vertical_threshold;
    ellipsoidThreshold1_ = params_.ellipsoidThreshold1;
    ellipsoidThreshold2_ = params_.ellipsoidThreshold2;
    ellipsoidThreshold3_ = params_.ellipsoidThreshold3;
    colorThreshold1_ = params_.colorThreshold1;
    colorThreshold2_ = params_.colorThreshold2;
    panEpsilon_ = params_.panEpsilon;
    tiltEpsilon_ = params_.tiltEpsilon;
    distEpsilon_ = params_.distEpsilon;
    minClusterSize_ = params_.minClusterSize;
}

void Segmentation::removeGround()
{
    if (rgbdCloud_->empty())
    {
        std::cout << "rgb cloud empty"  << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processingCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*rgbdCloud_, *processingCloud);
    obstacles_->clear();

    sortPointCloudByZ(processingCloud);

    double estLowestPoint = 0;

    for (int i = 0; i < numEstLPR_; i++)
    {
        estLowestPoint += processingCloud->points[i].z;
    }
    estLowestPoint /= numEstLPR_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr seedPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < processingCloud->size(); i++)
    {
        if (processingCloud->points[i].z < estLowestPoint + seedsThreshold_)
        {
            seedPoints->push_back(processingCloud->points[i]);
        }
    }

    // Construct covariance matrix
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*seedPoints, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*seedPoints, centroid, covariance);

    // singular value decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    // plane normal
    Eigen::Vector3f normal = eigenVectorsPCA.col(0);
    std::cout << "Found plane's normal: " << normal << std::endl;

    double centroidDistance = normal[0] * centroid[0] + normal[1] * centroid[1] + normal[2] * centroid[2];
    std::cout << "centroid distance: " << centroidDistance << std::endl;

    // TODO: can be parallelized
    for (int i = 0; i < static_cast<int>(rgbdCloud_->size()); i++)
    {
        pcl::PointXYZRGB point = rgbdCloud_->points[i];
        float x = point.x;
        float y = point.y;
        float z = point.z;
        auto distance = std::abs(normal[0] * x + normal[1] * y + normal[2] * z - centroidDistance);

        if (distance > distanceThreshold_)
        {
            obstacles_->push_back(point);
        }
        else
        {
        }
    }

    // Publish the obstacles
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*obstacles_, output_cloud_msg);
    output_cloud_msg.header.frame_id = params_.baseFrame;
    obstaclesPub_.publish(output_cloud_msg);
}

void Segmentation::removeBackground()
{
    if (obstacles_->empty())
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processingCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*obstacles_, *processingCloud);
    pcl::copyPointCloud(*obstacles_, *foreground_);

    while (processingCloud->size() > min_points_required_for_plane_)
    {
        // Create a new segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);

        // Set the input obstacles_ for this iteration
        seg.setInputCloud(processingCloud);
        // Perform plane segmentation
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            // No more planes found
            break;
        }

        // Extract the horizontal plane from the original point obstacles_
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(processingCloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // Extract outliers (non-horizontal points)
        extract.filter(*processingCloud);

        if (inliers->indices.size() < min_points_required_for_plane_)
        {
            // Not enough points to be a plane
            continue;
        }

        // Check if the plane is horizontal (e.g., normal vector points up)
        if (fabs(coefficients->values[2]) > vertical_threshold_ || fabs(coefficients->values[2]) < 1 - vertical_threshold_)
        {
            auto horizontal_indices = findPointsCloserToModel(
                foreground_, coefficients, plane_distance_threshold_);

            // Extract the horizontal plane from the original point obstacles_
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(foreground_);
            extract.setIndices(horizontal_indices);
            extract.setNegative(true);
            extract.filter(*foreground_);
        }
    }

    // Publish the modified point rgbdCloud_
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*foreground_, output_cloud_msg);
    output_cloud_msg.header.frame_id = params_.baseFrame;
    foregroundPub_.publish(output_cloud_msg);
}

void Segmentation::segment()
{
    std::cout << "Segmentation..." << std::endl;
    if (foreground_->empty())
    {
        std::cout << "foreground empty" << std::endl;
        return;
    }
    indexVector_ = std::vector<std::vector<int>>(int(360 / (ZED2Params.rgb_fov_x / ZED2Params.width)) + 1, std::vector<int>(int(180 / (ZED2Params.rgb_fov_y / ZED2Params.height)) + 1, -1));
    int cluster = 0;
    toSphere();

    // Define the binary mask Mat with the same size as the input data
    int rows = indexVector_.size();
    int cols = indexVector_[0].size();
    cv::Mat binaryMask(rows, cols, CV_8U, cv::Scalar(0)); // Initialize with zeros

    // // Draw the binary mask of foreground points
    // int count = 0;
    // // Iterate through the data and create the binary mask
    // for (int i = 0; i < rows; i++) {
    //     for (int j = 0; j < cols; j++) {
    //         if (indexVector_[i][j] != -1) {
    //             binaryMask.at<uchar>(i, j) = 200;
    //             count++;
    //         }
    //     }
    // }
    // // Save the binary mask as a PNG file
    // cv::imwrite("binary_mask.png", binaryMask);

    colors_ = std::vector<HSV>(sphereCloud_.points.size());
    visited_ = std::vector<bool>(sphereCloud_.points.size(), false);
    clusters_ = std::vector<int>(sphereCloud_.points.size(), -1);
    for (int i = 0; i < sphereCloud_.size(); i++)
    {
        if (visited_[i] == true)
        {
            continue;
        }

        std::vector<int> neighbors = regionQuery(i, true);
        if (neighbors.empty())
        {
            continue;
        }

        visited_[i] = true;
        int k = 0;
        while (k < neighbors.size())
        {
            std::vector<int> expandedNeighbors = regionQuery(neighbors[k], false);

            if (expandedNeighbors.size() > 0)
            {
                neighbors.insert(neighbors.end(), expandedNeighbors.begin(), expandedNeighbors.end());
            }
            k = k + 1;
        }

        if (neighbors.size() + 1 >= minClusterSize_)
        {
            clusters_[i] = cluster;
            for (int j = 0; j < neighbors.size(); j++)
            {
                clusters_[neighbors[j]] = cluster;
                visited_[neighbors[j]] = true;
            }
            cluster++;
        }
    }
    std::cout << "Number of clusters_: " << cluster << std::endl;

    // generate colored segments
    std::vector<RGB> colors;
    for (int i = 0; i < cluster; i++)
    {
        RGB color = {rand() % 255, rand() % 255, rand() % 255};
        colors.push_back(color);
    }

    coloredCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < sphereCloud_.size(); i++)
    {
        if (clusters_[i] == -1)
        {
            continue;
        }

        pcl::PointXYZRGB point = foreground_->points[i];
        point.r = colors[clusters_[i]].r;
        point.g = colors[clusters_[i]].g;
        point.b = colors[clusters_[i]].b;
        coloredCloud_->push_back(point);
    }

    // Publish the modified point rgbdCloud_
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*coloredCloud_, output_cloud_msg);
    output_cloud_msg.header.frame_id = params_.baseFrame;
    coloredCloudPub_.publish(output_cloud_msg);
}

void Segmentation::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    std::string target_frame = params_.baseFrame;
    try
    {
        sensor_msgs::PointCloud2 transformed_cloud;

        auto transformStamped = tf_buffer.lookupTransform(params_.baseFrame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));

        // tf_buffer.transform(*msg, transformed_cloud, target_frame);

        auto inputCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *inputCloud);
        rgbdCloud_->clear();
        tf::StampedTransform transform;
        tf::transformStampedMsgToTF(transformStamped, transform);

        pcl_ros::transformPointCloud(*inputCloud, *rgbdCloud_, transform);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Failed to transform point cloud: %s", ex.what());
    }
}

void Segmentation::toSphere()
{
    if (foreground_->empty())
    {
        std::cout << "No foreground point rgbdCloud_" << std::endl;
        return;
    }

    sphereCloud_.clear();
    int lastValue = 0;
    for (int i = 0; i < foreground_->size(); i++)
    {
        pcl::PointXYZRGB point = foreground_->points[i];
        float x = point.x;
        float y = point.y;
        float z = point.z;
        float r = std::sqrt(x * x + y * y + z * z);
        float theta = std::asin(z / r);
        theta += M_PI / 2; // 0 and M_PI
        float phi = std::atan2(y, x);
        if (phi < 0)
        {
            phi += 2 * M_PI;
        }
        phi *= 180 / M_PI;
        theta *= 180 / M_PI;
        point.x = r;
        point.y = theta;
        point.z = phi;
        sphereCloud_.push_back(point);
        auto panIdx = std::round(phi / (ZED2Params.rgb_fov_x / ZED2Params.width));     // finding pan index
        auto tiltIdx = std::round(theta / (ZED2Params.rgb_fov_y / ZED2Params.height)); // finding tilt index
        indexVector_[panIdx][tiltIdx] = i;
        lastValue = i;
    }
}

void Segmentation::checkNeighborhood(int index, int coreIndex, std::vector<int> &resultNeighbors)
{
    if (sphereCloud_.empty() || index < 0 || index >= sphereCloud_.size() || coreIndex < 0 || coreIndex >= sphereCloud_.size())
    {
        std::cout << "sphere size " << sphereCloud_.size() << " index " << index << " coreIndex " << coreIndex << std::endl;
        ROS_ERROR("Invalid index");
        return;
    }
    if (visited_[index] == true)
    {
        return;
    }
    clock_t begin = clock();
    if ((clusters_[index] != -1))
    {
        return;
    }
    HSV coreHsv;
    HSV indHsv;

    float panNeighbor = sphereCloud_.points[index].z;
    float tiltNeighbor = sphereCloud_.points[index].y;
    float distNeighbor = sphereCloud_.points[index].x;

    float panDist = 180 - fabs(fabs(sphereCloud_.points[coreIndex].z - panNeighbor) - 180); // calculate pan distance between two points
    float tiltDist = sphereCloud_.points[coreIndex].y - tiltNeighbor;                       // calculate tilt distance between two points
    float dist = distNeighbor - sphereCloud_.points[coreIndex].x;                           // calculate distance

    auto ellipsoidDist = ((panDist * panDist) / (panEpsilon_ * panEpsilon_)) + ((tiltDist * tiltDist) / (tiltEpsilon_ * tiltEpsilon_)) + ((dist * dist) / (distEpsilon_ * distEpsilon_)); // calculate ellipsoid distance

    if (ellipsoidDist <= ellipsoidThreshold1_)
    {
        // only unique indexes can be added
        resultNeighbors.push_back(index);
        visited_[index] = true;
        return;
    }

    // check if HSV dist is recorded before for both coreIndex and index
    if (colors_[index].h == 0 && colors_[index].s == 0 && colors_[index].v == 0)
    {
        RGB indRgb = {rgbdCloud_->points[index].r / 255.0, rgbdCloud_->points[index].g / 255.0, rgbdCloud_->points[index].b / 255.0};
        indHsv = rgbToHsv(indRgb);
        colors_[index] = indHsv;
    }
    else
        indHsv = colors_[index];
    if (colors_[coreIndex].h == 0 && colors_[coreIndex].s == 0 && colors_[coreIndex].v == 0)
    {
        RGB indRgb = {rgbdCloud_->points[coreIndex].r / 255.0, rgbdCloud_->points[coreIndex].g / 255.0, rgbdCloud_->points[coreIndex].b / 255.0};
        coreHsv = rgbToHsv(indRgb);
        colors_[coreIndex] = coreHsv;
    }
    else
        coreHsv = colors_[coreIndex];
    float colorDist = hsvDist(coreHsv, indHsv);

    // if ((ellipsoidDist <= elipsoid_thresh1) || (ellipsoidDist <= elipsoid_thresh2 && colorDist < color_thresh2) || (ellipsoidDist <= elipsoid_thresh3 && colorDist < color_thresh1))
    // if(ellipsoidDist <= elipsoid_thresh1)
    if ((colorDist < colorThreshold1_) || (ellipsoidDist <= ellipsoidThreshold2_ && colorDist < colorThreshold2_))
    {
        // only unique indexes can be added
        resultNeighbors.push_back(index);
        visited_[index] = true;
    }
}

std::vector<int> Segmentation::regionQuery(int iterator, bool core)
{
    std::vector<int> result;

    if (sphereCloud_.empty() || iterator < 0 || iterator >= sphereCloud_.size())
    {
        ROS_ERROR("Invalid index");
        return result;
    }

    auto panIdx = int(sphereCloud_.points[iterator].z / (ZED2Params.rgb_fov_x / ZED2Params.width));
    auto tiltIdx = int(sphereCloud_.points[iterator].y / (ZED2Params.rgb_fov_y / ZED2Params.height));
    for (int i = -panLimit_; i <= panLimit_; i++)
    {
        for (int j = -tiltLimit_; j <= tiltLimit_; j++)
        {
            if (i == 0 && j == 0)
            {
                continue;
            }

            int pan = panIdx + i;
            int tilt = tiltIdx + j;
            if (tilt < 0 || tilt >= indexVector_[0].size())
            {
                continue;
            }
            if (pan < 0)
            {
                pan += 360 / (ZED2Params.rgb_fov_x / ZED2Params.width);
            }
            if (pan >= 360 / (ZED2Params.rgb_fov_x / ZED2Params.width))
            {
                pan -= 360 / (ZED2Params.rgb_fov_x / ZED2Params.width);
            }
            auto index = indexVector_[pan][tilt];
            if (index == -1)
            {
                continue;
            }
            checkNeighborhood(index, iterator, result);
        }
    }
    return result;
}
