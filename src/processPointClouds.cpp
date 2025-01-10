// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <Eigen/Dense>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    // Create the filtering object, for downsampling the pointCloud
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_bounded (new pcl::PointCloud<PointT>);
    
    // Crop the point cloud for a certain region
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud (cloud_filtered);
    region.filter(*cloud_bounded);

    // remove the roof points
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud (cloud_bounded);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
     for (auto point : indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_bounded);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_bounded);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_bounded;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for (auto ind : inliers->indices){
        planeCloud->points.push_back(cloud->points[ind]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    if (inliers->indices.size () == 0){
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");

    }
    segResult = SeparateClouds(inliers,cloud);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;

    for (const auto& cluster : cluster_indices)
    {

        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        for (const auto& idx : cluster.indices) {
        
        cloud_cluster->push_back((*cloud)[idx]);

        } //*

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}




/* template <typename PointT>
BoxQ ProcessPointClouds<PointT>::ComputeOBB(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Ensure the cluster has points
    if (cluster->points.empty()) {
        throw std::runtime_error("Point cloud cluster is empty.");
    }

    // Step 1: Compute the centroid of the 2D points
    Eigen::Vector2f centroid(0, 0);
    for (const auto& point : cluster->points) {
        centroid += Eigen::Vector2f(point.x, point.y);
    }
    centroid /= static_cast<float>(cluster->points.size());

    // Step 2: Compute the covariance matrix
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const auto& point : cluster->points) {
        Eigen::Vector2f centered_point(point.x - centroid.x(), point.y - centroid.y());
        covariance += centered_point * centered_point.transpose();
    }
    covariance /= static_cast<float>(cluster->points.size());

    // Step 3: Compute eigenvectors and eigenvalues of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
    Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();

    // Step 4: Rotate points into the new coordinate system (aligned with eigenvectors)
    Eigen::Matrix2f rotation = eigenvectors.transpose();

    // std::numeric_limits- a standardized way to query various properties of arithmetic types 
    // max() - specifies the maximum possible value for the specified type 
    // By starting with std::numeric_limits<float>::max() for the minimum 
    // and std::numeric_limits<float>::lowest() for the maximum,
    // you guarantee that any point in the dataset will replace the initial value
    // Eigen::Vector2f min_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    // Eigen::Vector2f max_point(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());

    Eigen::Vector2f min_point(0,0);
    Eigen::Vector2f max_point(0,0);

    for (const auto& point : cluster->points) {
        Eigen::Vector2f rotated_point = rotation * Eigen::Vector2f(point.x - centroid.x(), point.y - centroid.y());
        min_point = min_point.cwiseMin(rotated_point);
        max_point = max_point.cwiseMax(rotated_point);
    }

    // Step 5: Compute the OBB properties
    Eigen::Vector2f half_lengths = (max_point - min_point) / 2.0f; // Half lengths of the OBB
    Eigen::Vector2f center_rotated = (min_point + max_point) / 2.0f; // Center in rotated space
    Eigen::Vector2f center = eigenvectors * center_rotated + centroid; // Center in original space

    // Step 6: Populate the BoxQ structure
    BoxQ obb;
    obb.bboxTransform = Eigen::Vector3f(center.x(), center.y(), 0.0f); // Center with z = 0
    obb.bboxQuaternion = Eigen::Quaternionf(Eigen::AngleAxisf(atan2(eigenvectors(1, 0), eigenvectors(0, 0)), Eigen::Vector3f::UnitZ()));
    obb.cube_length = 2.0f * half_lengths.x(); // Major axis length
    obb.cube_width = 2.0f * half_lengths.y();  // Minor axis length
    obb.cube_height = 0.0f;                    // Fixed for 2D points

    return obb;
}
 */
template <typename PointT>
BoxQ ProcessPointClouds<PointT>::ComputeOBB(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Ensure the cluster has points
    if (cluster->points.empty()) {
        throw std::runtime_error("Point cloud cluster is empty.");
    }

    // Step 1: Compute the centroid of the 3D points
    Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
    for (const auto& point : cluster->points) {
        centroid += Eigen::Vector3f(point.x, point.y, point.z);
    }
    centroid /= static_cast<float>(cluster->points.size());

    // Step 2: Compute the covariance matrix
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& point : cluster->points) {
        Eigen::Vector3f centered_point(point.x - centroid.x(), point.y - centroid.y(), point.z - centroid.z());
        covariance += centered_point * centered_point.transpose();
    }
    covariance /= static_cast<float>(cluster->points.size());

    // Step 3: Compute eigenvectors and eigenvalues of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();

    // Ensure a right-handed coordinate system for the eigenvectors
    if (eigenvectors.determinant() < 0) {
        eigenvectors.col(2) = -eigenvectors.col(2);
    }

    // Step 4: Rotate points into the new coordinate system (aligned with eigenvectors)
    Eigen::Matrix3f rotation = eigenvectors.transpose();
    Eigen::Vector3f min_point(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    );
    Eigen::Vector3f max_point(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest()
    );

    for (const auto& point : cluster->points) {
        Eigen::Vector3f rotated_point = rotation * (Eigen::Vector3f(point.x, point.y, point.z) - centroid);
        min_point = min_point.cwiseMin(rotated_point);
        max_point = max_point.cwiseMax(rotated_point);
    }

    // Step 5: Compute the OBB properties
    Eigen::Vector3f half_lengths = (max_point - min_point) / 2.0f; // Half lengths of the OBB
    Eigen::Vector3f center_rotated = (min_point + max_point) / 2.0f; // Center in rotated space
    Eigen::Vector3f center = eigenvectors * center_rotated + centroid; // Center in original space

    // Step 6: Populate the BoxQ structure
    BoxQ obb;
    obb.bboxTransform = center; // Center in original space
    obb.bboxQuaternion = Eigen::Quaternionf(eigenvectors); // Rotation matrix to quaternion
    obb.cube_length = 2.0f * half_lengths.x(); // Length along principal axis
    obb.cube_width = 2.0f * half_lengths.y();  // Width along secondary axis
    obb.cube_height = 2.0f * half_lengths.z(); // Height along tertiary axis

    return obb;
}




template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}