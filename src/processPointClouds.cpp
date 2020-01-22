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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) // minPoint x, y, z and maxPoint x, y, z
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Create the filtered object then downsample the dataset using a leaf size of 0.2m
    // so there is only a single point in each voxel
    pcl::VoxelGrid<PointT> vg;  // create a voxel grid
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);  // create a new point cloud to put our filtered results into
    //std::cout << typeid(vg).name() << endl;
    vg.setInputCloud(cloud);  // give the voxel grid the cloud
    vg.setLeafSize(filterRes, filterRes, filterRes);  // define the cell size for the voxel grid
    vg.filter(*cloudFiltered);  // after filtering, put the results in cloudFiltered

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);  // create a new point cloud called cloudRegion

    pcl::CropBox<PointT> region(true);  // set region to true because you're dealing with points inside the cropbox
    region.setMin(minPoint);  // set min from the arguments passed into this function
    region.setMax(maxPoint);  // set max from the arguments passed into this function
    region.setInputCloud(cloudFiltered);  // here we set the input cloud to cloudFiltered
    region.filter(*cloudRegion);  // here we put the results of CropBox into cloudRegion

    // Now let's try to remove the roof points.  Those points are static and don't really tell us anything
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);  // set region to true because you're dealing with points inside the cropbox
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));  // found via playing around with the code and visuals
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));  // found via playing around with the code and visuals
    roof.setInputCloud(cloudRegion);  // give it the cloudRegion to act on
    roof.filter(indices);  // save the results into indices, which are the points inside of cloudRegion that fit inside the defined box (the roof of the vehicle)

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};  // create a PointIndicies object
    for (int point : indices)  // itterate through the indices vector (all the points we want to remove from cloudRegion)
        inliers -> indices.push_back(point);  // take those inliers and push them into inliers
    
    // Now, pull the rooftop inliers out of the cloudRegion data set
    pcl::ExtractIndices<PointT> extract;  // create an extract 
    extract.setInputCloud(cloudRegion);  // set the input cloud to cloudRegion
    extract.setIndices(inliers);  // give it inliers, the indices of points
    extract.setNegative(true);  // setting Negative to "true" means that we will be removing the inliers points
    extract.filter(*cloudRegion);  // push the final results into cloudRegion

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

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

    // Create the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>); // use template <PointT> 
    tree -> setInputCloud(cloud);  // feed the cloud (or agrument) into the clustering function, the KD-Tree called tree

    std::vector<pcl::PointIndices> clusterIndices;  // create the cluster indicies
    pcl::EuclideanClusterExtraction<PointT> ec;  // create an object called ec
    ec.setClusterTolerance(clusterTolerance);  // cluster tolerence is passed in from this functions header
    ec.setMinClusterSize(minSize);  // min size is passed in from this functions header
    ec.setMaxClusterSize(maxSize);  // max size is passed in from this functions header
    ec.setSearchMethod(tree);  // here I tell the ec object to use the tree from above
    ec.setInputCloud(cloud);  // here I set the ec object with the cloud
    ec.extract(clusterIndices);  // here I generate my cluster indicies

    // Now I'm set to go through the cluster indicies and create some point clouds
    // Each point cloud is goint to be a different cluster
    for (pcl::PointIndices getIndices: clusterIndices)  // itterate through clusterIndices, and the type for those is pcl::PointIndices ... a vector of pcl::PontIndices
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);  // create a new obstacle cloud, cloud cluster

        for (int index : getIndices.indices) // now I itterate through the indices (the indexes)
            cloudCluster -> points.push_back (cloud -> points[index]);  // Take each indice and push it back into the obstacle cluster that I just created .. the cloudCluster
        
        cloudCluster -> width = cloudCluster -> points.size();  // set up the width and height
        cloudCluster -> height = 1;
        cloudCluster -> is_dense = true;

        clusters.push_back(cloudCluster);  // now push it back into my clusters, which is my vector of point clouds
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