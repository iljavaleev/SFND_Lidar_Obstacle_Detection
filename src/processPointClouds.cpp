// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <random>
#include <cmath>
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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filtered{new typename pcl::PointCloud<PointT>()};
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filtered);
    
    typename pcl::PointCloud<PointT>::Ptr reg{new typename pcl::PointCloud<PointT>()};
    pcl::CropBox<PointT> box(true);
    
    box.setMin(minPoint);
    box.setMax(maxPoint);
    box.setInputCloud(filtered);
    box.filter(*reg);
    
    //remove roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(reg);
    roof.filter(indices);
    
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (auto i: indices)
        inliers->indices.push_back(i);
    
    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(reg);
    ex.setIndices(inliers);
    ex.setNegative(true);
    ex.filter(*reg);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    
   
    return reg;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  
    typename pcl::PointCloud<PointT>::Ptr obstCloud{new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr planeCloud{new pcl::PointCloud<PointT>()};
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    
    for (auto i: inliers->indices){
        planeCloud->points.push_back(cloud->points.at(i));
    }
//    extract.setNegative(false);
//    extract.filter(*planeCloud);


    extract.setNegative(true);
    extract.filter(*obstCloud);

    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    

    

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0){
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
       
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RANSACSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, int(cloud->points.size())-1);
    
    PointT p1, p2, p3, inl;
    double A, B, C, D, distance;
    int max{};
    std::unordered_set<int> inliersResult;
    
    
    for (int i{}; i < maxIterations; i++){
        auto p1 = cloud->points.at(distr(gen));
        auto p2 = cloud->points.at(distr(gen));
        auto p3 = cloud->points.at(distr(gen));
        
        double x1{p1.x}, x2{p2.x}, x3{p3.x};
        double y1{p1.y}, y2{p2.y}, y3{p3.y};
        double z1{p1.z}, z2{p2.z}, z3{p3.z};
        
        double ii{(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1)};
        double jj{(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1)};
        double kk{(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1)};
        
        
        A = ii;
        B = jj;
        C = kk;
        D = -(ii * x1 + jj * y1 + kk * z1);
        
        std::unordered_set<int> inls;
        int count{};
        
        for(int j{}; j<cloud->points.size(); j++){
            inl = cloud->points.at(j);
            distance = std::fabs(A * inl.x + B * inl.y + C * inl.z + D)/std::sqrt(A * A + B * B + C * C);
            if (distance <= distanceThreshold){
                inls.insert(j);
                ++count;
            }
        }
        
        if (count > max){
            max = count;
            inliersResult = inls;
        }
    }
    
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);
    
    pcl::EuclideanClusterExtraction<PointT> ec;
    std::vector<pcl::PointIndices> cluster_indices;
    
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    
    ec.extract(cluster_indices);
    
    for (const auto &cluster: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new typename pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back(cloud->points.at(idx));
        }
        cloud_cluster->width = cloud_cluster->size();
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

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster){
    // compute the centroid (c0, c1, c2) and the normalized covariance
    //compute PCA
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    
    Eigen::Matrix3f covMatrix;
    pcl::computeCovarianceMatrixNormalized (*cluster, pcaCentroid, covMatrix);
    //compute the eigenvectors e0, e1, e2.  (e0, e1, e0 X e1)
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covMatrix, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigPca = eigen_solver.eigenvectors();
    eigPca.col(2) = eigPca.col(0).cross(eigPca.col(1));
    
    //move the points in that RF
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigPca.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected {new typename pcl::PointCloud<PointT>};
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    
    //compute the max, the min and the center of the diagonal
    // Get the minimum and maximum points of the transformed cloud.
    PointT min, max;
    pcl::getMinMax3D(*cloudPointsProjected, min, max);
    const Eigen::Vector3f meanDiagonal = 0.5f * (max.getVector3fMap() + min.getVector3fMap());
    
    const Eigen::Quaternionf bboxQuaternion(eigPca);
    const Eigen::Vector3f bboxTransform{eigPca * meanDiagonal + pcaCentroid.head<3>()};
    
    
    return BoxQ{bboxTransform, bboxQuaternion, max.x - min.x, max.y - min.y, max.z - min.z};
    
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
std::vector<std::__fs::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<std::__fs::filesystem::path> paths(std::__fs::filesystem::directory_iterator{dataPath}, std::__fs::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
