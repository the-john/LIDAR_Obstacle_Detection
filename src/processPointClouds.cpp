// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) // passing in a pointer to inliers and a pointer to cloud; made in the SegmentPlane function below from the SeparateCloud call
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());  // create a new pointer for the obstCloud
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());  // create a new pointer for the planeCloud

    for (int index : inliers -> indices)  // inliers comes from SeperateClouds inliers below on line 99, passed into this function from above (line 42)
        planeCloud -> points.push_back(cloud -> points[index]);  // pass in the inliers for the planeCloud

    pcl::ExtractIndices<PointT> extract;  // using the extract object and a pointer (PointT)
    extract.setInputCloud (cloud);  // give extract the reference cloud
    extract.setIndices (inliers);  // give all of the inliers in the reference cloud
    extract.setNegative (true);     // set all of those inliers to negative
    extract.filter (*obstCloud);  // de-reference the pointer obstCloud and pass it into filter.  Now, all the points in our reference "cloud" that are not the inliers are kepted.  Inliers points are removed from the reference cloud.  So now, only obstacles are remaining.

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);  // now return the results from the lines above
    return segResult;
}



// This function accepts a point cloud, maximum itterations, and distance tolerance as arguments.
// Segmentation uses an itterative process.  More itterations have a chance of returning better results but take longer.
// Segmentation algorithm fits a plane to the points and uses the distance tolerance to decide which point belong to that plane.
//  A larger tollerance includes more points on the plane.
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;

    // TODO:: Fill in this function to find inliers for the cloud.
    // Here we segment the cloud into two parts, the drivable plane and the various obstacles around us
    pcl::SACSegmentation<PointT> seg;  // create an object called seg.  Using PointT because our PointProcessor defines this template point for us.  This way we can pass in any type of point.
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};  // define inliers from pcl type PointIndicies.  We use this later to seperate out the point cloud and break it up into two pieces.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};  // here we define the coefficients of our model (SACMODEL_PLANE); can use these coefficients to render the plane in the plc viewr.

    seg.setOptimizeCoefficients(true);  // this helps us to try and get the best model possible
    seg.setModelType(pcl::SACMODEL_PLANE);  // this defines what kind of model we will use
    seg.setMethodType(pcl::SAC_RANSAC);  // Random Sample Concensus ... this is the secret sauce that makes this all work
    seg.setMaxIterations(maxIterations);  // this argument is passed in through this function, key to RANSAC itself
    seg.setDistanceThreshold(distanceThreshold);  // this argument is passed in through this function, key to RANSAC itself

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);  // we want to do segmentation on our cloud, which is passed into the function via typename pcl::PointCloud<PointT>::Ptr cloud
    seg.segment (*inliers, *coefficients);  // now we generate the inliers and coefficients (though here, we won't actually be using the coefficients for anything)
                                            // but we use the inliers to seperate the point cloud into two pieces
                                            // we de-reference inliers here (*inliers) because it references a pointer from line 63 above
                                            // inlier pointer points to all the list of indicies that belong to the plane we found by doing RANSAC
    if (inliers -> indices.size() == 0)  // pass in inliers via reference (created inliers up on line 63)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        }
    
    // We now have our inliers and can start seperating our cloud.
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);  // see line 50 above

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;   // we are returning a pair of point clouds.  
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

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