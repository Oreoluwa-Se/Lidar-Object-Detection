
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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

// Fuction for real pcd
template<typename PointT>
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<PointT>* pointProcessor,
 const typename pcl::PointCloud<PointT>::Ptr& inputCloud, const bool custom=false, const bool use_rot=false){
    
    // filter cloud
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud= pointProcessor->FilterCloud(inputCloud, 0.3,
    	Eigen::Vector4f (-20, -7, -4, 1), Eigen::Vector4f (20, 7, 4, 1));
    //renderPointCloud(viewer, filtered_cloud, "filterCloud");

    // segment into road and obstacles
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filtered_cloud, 30, 0.3, custom);    
    
    //Loading only the road. obstacles loaded after clustering
    renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud", Color(1,1,0));
    renderPointCloud(viewer, segmentCloud.second, "Plane Cloud", Color(0,1,0));

    // cluster the remaining bits - obstacel cloud, distance tolerance, minimum points, maximum distance 
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.6, 30, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,1), Color(1,1,0), Color(1,1,1)};

    for(typename pcl::PointCloud<PointT>::Ptr cluster : cloudClusters) {
          //std::cout << "cluster size: "<< pointProcessor->numPoints(cluster) << endl;
          //renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);          

          // bounding boxes -> using the rotated or not
          if (!use_rot){
          	Box box = pointProcessor->BoundingBox(cluster);
          	renderBox(viewer, box, clusterId);

          }	else {
          	BoxQ box = pointProcessor->RotatedBoundingBox(cluster);
          	renderBox(viewer, box, clusterId);
          	
      	}
      	
        ++clusterId;
    }

}


//void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
//{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS -> redner scene set to false becasue
    // we want to visualize point cloud only
  //  bool renderScene = false;
   // std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor -> pointer type
  //  Lidar* car_lid = new Lidar(cars, 0);
    
    // renders rays
    //renderRays(viewer, car_lid ->position, car_lid->scan());

    // renders point cloud
    //renderPointCloud(viewer, car_lid->scan(), "input");

    // instantiating point processor -> 2 options
  //  ProcessPointClouds<pcl::PointXYZ> pointProcessor; // stack setup
//
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(car_lid->scan(), 300, 0.2);
	//renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
	//renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

   // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.625, 15, 30);

    //int clusterId = 0;
    //std::vector<Color> colors = {Color(1,0,1), Color(1,1,0), Color(1,1,1)};

  //  for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
   // {
   //       std::cout << "cluster size ";
   //       pointProcessor.numPoints(cluster);
   //       renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          

          // bounding boxes
          //Box box = pointProcessor->BouningBox(cluster);
          //renderBox(viewer, box, clusterId);

    //      ++clusterId;
  //  }
    

  
//}


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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // loading pcl files 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    // instantiate the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
    	// Clear viewer
    	viewer-> removeAllPointClouds();
    	viewer-> removeAllShapes();

    	// load the pcd and run obstacle detection
    	inputCloudI = pointProcessor->loadPcd((*streamIterator).string());
    	cityBlock(viewer, pointProcessor, inputCloudI, false, true);

    	// stream iterator operations
    	streamIterator++;

    	// loop pcd files
    	if(streamIterator == stream.end())
    		streamIterator = stream.begin();

    	viewer->spinOnce ();
    } 

    
}