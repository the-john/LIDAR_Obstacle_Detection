// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)  // with additional arguments for the pointProcessor that I'm passing in so that I don't have to create it everytime inside the function
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)  // the argument used is our viewer
{
    /*
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;  // create pointProcesor that uses XYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");  // load up the point cloud data
    
    renderPointCloud(viewer, inputCloud, "inputCloud");
    */
        
    Eigen::Vector4f min (-15, -6.25, -3, 1.0);  // minimum size for the box (or region of interst); x is red, y is green, z is blue axis in image
    Eigen::Vector4f max (20, 7.65, 10, 1.0);  // maximum size for the box (or region of interest); x is red, y is green, z is blue axis in image
   
    //ProcessPointClouds<pcl::PointXYZI> pointProcessor;  // create pointProcesor that uses XYZI; not needed now because I'm passing it in
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");  // load up the point cloud data; no longer need to load this cloud
    //pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, 0.3, min, max);
    inputCloud = pointProcessor.FilterCloud(inputCloud, 0.3, min, max);
    
    //std::cout << "original size: " << inputCloud -> width * inputCloud -> height << endl;
    //std::cout << "filtered size: " << filterCloud -> width * filterCloud -> height << endl;

    // Segment out the ground plane from the obstacles
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.13); // call the point processor, 100 itterations, distance of 0.13m.  This line creates our pair, obstCloud and planeCloud
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 25, 0.3);
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud");

    // Render the road as green, and all of the obstacles as red
	renderPointCloud(viewer, segmentCloud.second, "inliers", Color(0,1,0));
  	//renderPointCloud(viewer, segmentCloud.first, "outliers", Color(1,0,0));
	
    ///*
    // Place a purple box to represent the 3D space that our car takes up (see render.cpp for details)
    Box meBox;  // set the dimensions of the box
    meBox.x_min = -2.0;
    meBox.y_min = -1.0;
    meBox.z_min = -1.75;
    meBox.x_max = 2.0;
    meBox.y_max = 1.0;
    meBox.z_max = 0;
    renderBox(viewer, meBox, 0, Color(1, 0, 1));  // color setting is for purple
    //renderBox(viewer, meBox, 0);
    //*/


    // Cluster the obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.75, 7, 1000); // ".first" part is where obstacles points are, ".second" part is where the road points are  ... here we grap the first part, obstacles.  Hyper-parameters: 0.1 for distance tollerance, 3 for min # of points to be considered a cluster, 100 for max # of points to be considered a cluster
    
    // Now itterate through my vector of point clouds
    int clusterId = 1;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(1, 1, 1), Color(0, 1, 1), Color(1, 0, 1)};  // set up colors for clusters of point clouds by creating a color list, to be used later.  Red, Yellow, and Blue
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)  // grab the point cloud and call it cluster ... from the cloudClusters
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);  // render pointcloud, feed it cluster and give it the name obstCloud, then give it a color

        // Put a colored box around the cluster
        Box clusterBox = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, clusterBox, clusterId, colors[clusterId%colors.size()]);
        //renderBox(viewer, clusterBox, clusterId);

        ++clusterId;  // so I can itterate through the vector of point clouds
        //clusterId++;
    }

}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;   //true if you want to render our scene
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2); // call the point processor, 100 itterations, distance of 0.2m.  This line creates our pair, obstCloud and planeCloud
    
    //if (render_obst)
        //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));  // show the obstacle cloud by pulling the first element of segmentCloud
    //if (render_plane)
        //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0)); // show the plane cloud by pulling the second element of segmentCloud (color is R, G, B)

    // Call our pointProcessor Clustering function
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 2.0, 3, 30); // first part is obstacles, second part is road points  ... here we grap the first part, obstacles.  Hyper-parameters: 1.0 for distance tollerance, 3 for min # of points to be considered a cluster, 30 for max # of points to be considered a cluster
    
    // Now itterate through my vector of point clouds
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};  // set up colors for clusters of point clouds by creating a color list, to be used later.  Red, Yellow, and Blue
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)  // grab the point cloud and call it cluster ... from the cloudClusters
    {
        //if (render_clusters)
        //{
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);  // render pointcloud, feed it cluster and give it the name obstCloud, then give it a color
        //}

        //if (render_box)
        //{
            // Put a box around the rendered point cloud
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        //}

        ++clusterId;  // so I can itterate through the vector of point clouds
    }
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud");
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    
    // Create pointProcessor cloud.  call it pointProcessorI because it is dealing with intensity now
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;  // creating this on the stack (so every time I'm using the function, use the dot operator)
    // Now create a stream coming out of "data_1" folder
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    // Create a stream iterator and start from the beginning
    auto streamIterator = stream.begin();
    // Create a placeholder for the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    /*
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
    */

    while (!viewer -> wasStopped())
    {
        // Clear the viewer
        viewer -> removeAllPointClouds();
        viewer -> removeAllShapes();

        // Load point cloud data and run the obstacle detection process
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());  // dereference the streamIterator that contains the path ... convert it to a string.  This loads up the point cloud data from the "data_1" file
        cityBlock(viewer, pointProcessorI, inputCloudI);  // now call cityBlock with the pointProcessorI created on line 191 above, and use the pointCloudI that we just loaded up

        streamIterator++;  // increment the stream iterator so we can move to the next frame in the file "data_1"

        if (streamIterator == stream.end())
            streamIterator = stream.begin();    // just keep looping the data

        viewer -> spinOnce();
    }

}