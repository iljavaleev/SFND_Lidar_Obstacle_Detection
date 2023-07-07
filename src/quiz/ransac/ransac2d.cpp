/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
#include <cmath>
#include <unordered_set>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("/Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

//std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
//{
//	std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_int_distribution<> distr(0, int(cloud->points.size())-1);
//
//    pcl::PointXYZ p1, p2, inl;
//    double A, B, C, distance;
//    int max{};
//    std::unordered_set<int> inliersResult;
//
//
//    for (int i{}; i < maxIterations; i++){
//        auto p1 = cloud->points.at(distr(gen));
//        auto p2 = cloud->points.at(distr(gen));
//        A = p1.y - p2.y;
//        B = p2.x - p1.x;
//        C = p1.x * p2.y - p2.x * p1.y;
//
//        std::unordered_set<int> inls;
//        int count{};
//
//        for(int j{}; j<cloud->points.size(); j++){
//            inl = cloud->points.at(j);
//            distance = std::fabs(A * inl.x + B * inl.y + C)/std::sqrt(A * A + B * B);
//            if (distance <= distanceTol){
//                inls.insert(j);
//                ++count;
//            }
//        }
//
//        if (count > max){
//            max = count;
//            inliersResult = inls;
//        }
//    }
//
//	return inliersResult;
//
//}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, int(cloud->points.size())-1);
    
    pcl::PointXYZ p1, p2, p3, inl;
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
            if (distance <= distanceTol){
                inls.insert(j);
                ++count;
            }
        }
        
        if (count > max){
            max = count;
            inliersResult = inls;
        }
    }
    
    return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
