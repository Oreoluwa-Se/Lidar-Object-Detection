// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "helper/ransac3d.h"
#include <unordered_set>


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

    // storage
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>);

    // create voxel grid
    pcl::VoxelGrid<PointT> vox_grid;
    vox_grid.setInputCloud(cloud);
    vox_grid.setLeafSize(filterRes, filterRes, filterRes);
    vox_grid.filter(*filteredCloud);

    // cloud region
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*cloudRegion);

    // removing points in the from the roof
    std::vector<int> roofPoints;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-2, -2, -1, 1));
    roof.setMax(Eigen::Vector4f (3, 2, 1, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(roofPoints);


    // loop through roof points
    pcl::PointIndices::Ptr inliers {new::pcl::PointIndices};
    for (int point : roofPoints)
        inliers->indices.push_back(point);

    // Extract method in effect
    pcl::ExtractIndices<PointT> extract_;
    extract_.setInputCloud(cloudRegion);
    extract_.setIndices(inliers);
    extract_.setNegative(true);
    extract_.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // create number of required clouds to be segmented.. here 2, 
    //one for road and one for obstacles
    typename pcl::PointCloud<PointT>::Ptr cloudObst(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudPlane(new pcl::PointCloud<PointT>());

    // push into plane cloud -> push into the points vector
    for(int i : inliers->indices){
        cloudPlane->points.push_back(cloud->points[i]);
    }

    // Extract method in effect
    pcl::ExtractIndices<PointT> extract_;
    extract_.setInputCloud(cloud);
    extract_.setIndices(inliers);
    extract_.setNegative(true);
    extract_.filter(*cloudObst);

    // return result
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudObst, cloudPlane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, float distanceThreshold, const bool custom)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // Create coefficients and point indicies
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    if (!custom){
        // create the segmentation object
        pcl::SACSegmentation<PointT> seg;

        // initialize segmentation paramentes
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);

        // Seperates largest segment from the input cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
    } else {
        // using unordered set to avoid duplicates
        std::unordered_set<int> inliers = Ransac3D<PointT>::Ransac(cloud, maxIterations, distanceThreshold);

    }

    if (inliers->indices.size () == 0){
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    } 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // performing euclidean clustering
    // create the kd tree object for the search method
    typename pcl::search::KdTree<PointT>::Ptr tree_ (new pcl::search::KdTree<PointT>);
    tree_->setInputCloud(cloud);

    // create a vectore of cluster indicies
    std::vector<pcl::PointIndices> cluster_indices;

    // creates an extraction objects
    pcl::EuclideanClusterExtraction<PointT> ec;

    // set attributes
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree_);
    ec.setInputCloud(cloud);

    // extracts clusters from the point cloud
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        // create an instance of the point cloud
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

        // looping through the indices for each cluster
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            cloud_cluster->push_back((*cloud)[*pit]);
        }

        // fill cluster parameters
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense=true;

        // push into the cluster vector
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

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::RotatedBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // solution from http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
    // step 1 PCA
    typename pcl::PointCloud<PointT>::Ptr cloudSegmented = cluster;

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); 
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Box
    BoxQ box;

    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
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