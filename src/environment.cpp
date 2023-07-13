/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
//#include "quiz/ransac/ransac2d.cpp"
//#include "quiz/cluster/cluster.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, bool renderScene = false, bool render_clusters = false, bool render_box = false, bool render_min_max = false)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    
     //with/without cars
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    
    Lidar *lidar_ptr = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar_ptr->scan();

    renderPointCloud(viewer, cloud, "Some name");
    
    ProcessPointClouds<pcl::PointXYZ> *process_ptr{new ProcessPointClouds<pcl::PointXYZ>()};
    
    auto segmentCloud = process_ptr->SegmentPlane(cloud, 100, 0.2);
//
//    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = process_ptr->Clustering(segmentCloud.first, 0.5, 0, 30);
    int clusterId{};
    std::vector<Color> colors{Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0 ), Color(0.1, 1, 0.1), Color(0, 1, 0), Color(0, 1, 1), Color(1, 1, 1), Color(0.5, 0.5, 1), Color(0.5, 1, 1), Color(0.5, 0.5, 1), Color(0, 0.4, 0.3)};
    
    for (auto c:cloudClusters){
        if (render_clusters){
            std::cout << "cluster size ";
            process_ptr->numPoints(c);
            renderPointCloud(viewer, c, "obstCloud" + std::to_string(clusterId), colors.at(clusterId));
        }
        if (render_box){
            Box box = process_ptr->BoundingBox(c);
            renderBox(viewer, box, clusterId);
        }
        if (render_min_max){
            BoxQ box = process_ptr->BoundingBoxQ(c);
            renderBox(viewer, box, clusterId);
        }
        clusterId++;
    }
    
    delete process_ptr;
    delete lidar_ptr;
  
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *process_ptr, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud){
    
    auto filterCloud = process_ptr->FilterCloud(inputCloud, 0.3 , Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 30, 8, 1, 1));
       
    auto segmentCloud = process_ptr->RANSACSegmentPlane(filterCloud, 25, 0.3);
    
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = process_ptr->Clustering(segmentCloud.second, 0.53, 10, 300);

    int clusterId{};
    std::vector<Color> colors{Color(1,0,1), Color(0,1,0.5), Color(0,0,1), Color(1,1,0)};

    bool render_clusters{true},render_box{true},render_min_max{false};

    for (auto c:cloudClusters){
        if (render_clusters){
            std::cout << "cluster size ";
            process_ptr->numPoints(c);
            renderPointCloud(viewer, c, "obstCloud" + std::to_string(clusterId), colors.at(clusterId % colors.size()));
        }
        if (render_box){
            Box box = process_ptr->BoundingBox(c);
            renderBox(viewer, box, clusterId);
        }
        if (render_min_max){
            BoxQ box = process_ptr->BoundingBoxQ(c);
            renderBox(viewer, box, clusterId);
        }
        clusterId++;
    }
    
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer, false, true, false, true);
    
    ProcessPointClouds<pcl::PointXYZI> *process_ptr{new ProcessPointClouds<pcl::PointXYZI>()};
    
    std::vector<std::__fs::filesystem::path> stream{process_ptr->streamPcd("/Users/ilavaleev/Dev/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2")};
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    
    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
            
        inputCloudI = process_ptr->loadPcd((*streamIterator).string());
        cityBlock(viewer, process_ptr, inputCloudI);
        
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        
        viewer->spinOnce();
        pcl_sleep(0.5);
    } 
}
