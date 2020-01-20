/* \author John Oliver via Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>
#include <vector>

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
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;  // this is used to hold my best inliers (whatever has the highest number of inliers)
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while (maxIterations--)  // while the number of maxIterations is not zero
	{
		// Randomly pick two points to create our line from
		std::unordered_set<int> inliers;  // a new unordered set of inliers randomly sampled from cloud; set will only contain unique elements (so we don't pick the same index set twice in a row).  inliers is an int, so it's holding an index to some point from cloud
		while (inliers.size() < 2)
			inliers.insert(rand()%(cloud -> points.size()));  // insert a point randomly.  To control the size of the number, use modulo %.  This will force the random number to be between zero and the point size from cloud.  Store it into inliers.

		float x1, y1, x2, y2;  // so we want to see what a point in the cloud looks like, and inliers is holding that information

		auto itr = inliers.begin();  // lets pull the first value out of inliers.  itr is a pointer and it's pointing to the very beginning of inliers
		x1 = cloud -> points[*itr].x; // we can see what that value is by de-referencing that pointer to the very beginning of inliears (the *itr becomes the index into inliers)
		y1 = cloud -> points[*itr].y;  // we are indexing into cloud and grabbing the the y value of that index
		itr++; // increment the pointer into inliers to get the second point
		x2 = cloud -> points[*itr].x;
		y2 = cloud -> points[*itr].y;

		// Now calculate the A, B, and C values for the coefficients for our line equation
		float a = (y1 - y2);
		float b = (x2 - x1);
		float c = (x1 * y2 - x2 * y1);

		for (int index = 0; index < cloud -> points.size(); index++) // now, let's itterate through the points
		{
			if (inliers.count(index) > 0)  // checking to see if the element (index) exists in inliers or not
				continue;

			pcl::PointXYZ point = cloud -> points[index];  // if the point isn't part of our model (the line), then calculate the distance of the point from the line and see if it is in the threshold or not
			float x3 = point.x;  // get the x member from point
			float y3 = point.y;  // get the y member from point

			float d = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);  // now, calculate the distance of this point from the line.  Remember to do "float absolute value" (fabs) so there is no rounding and wierd results

			if (d <= distanceTol)  // if d is less than or equal to the tollerance, then insert it into inliers (our un-ordered set)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size())  // if our new and random inliers is bigger than our reference inliersResult, then replace inliersResult with inliers.  inliersResult starts out as zero.
		{
			inliersResult = inliers;
		}

	}
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;  // this is used to hold my best inliers (whatever has the highest number of inliers)
	srand(time(NULL));

	while (maxIterations--)  // while the number of maxIterations is not zero
	{
		// Randomly pick two points to create our line from
		std::unordered_set<int> inliers;  // a new unordered set of inliers randomly sampled from cloud; set will only contain unique elements (so we don't pick the same index set twice in a row).  inliers is an int, so it's holding an index to some point from cloud
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud -> points.size()));  // insert a point randomly.  To control the size of the number, use modulo %.  This will force the random number to be between zero and the point size from cloud.  Store it into inliers.

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;  // so we want to see what a point in the cloud looks like, and inliers is holding that information

		auto itr = inliers.begin();  // lets pull the first value out of inliers.  itr is a pointer and it's pointing to the very beginning of inliers
		x1 = cloud -> points[*itr].x; // we can see what that value is by de-referencing that pointer to the very beginning of inliears (the *itr becomes the index into inliers)
		y1 = cloud -> points[*itr].y;  // we are indexing into cloud and grabbing the the y value of that index
		z1 = cloud -> points[*itr].z;
		itr++; // increment the pointer into inliers to get the second point
		x2 = cloud -> points[*itr].x;
		y2 = cloud -> points[*itr].y;
		z2 = cloud -> points[*itr].z;
		itr++; // increment the pointer into inliers to get the third point
		x3 = cloud -> points[*itr].x;
		y3 = cloud -> points[*itr].y;
		z3 = cloud -> points[*itr].z;

		// Use point1 as a reference and define two vectors on the plane v1 and v2
		// 	 Vector v1 travels from point1 to point2
        int xv1 = x2 - x1;
        int yv1 = y2 - y1;
        int zv1 = z2 - z1;
		std::vector<int> v1{xv1, yv1, zv1};
		//   Vector v2 travels from point1 to point3
        int xv2 = x3 - x1;
        int yv2 = y3 - y1;
        int zv2 = z3 - z1;
		std::vector<int> v2{xv2, yv2, zv2};

		// Take the cross product of v1 and v2 to find the normal vector to the plane
		int xvn = (yv1 * zv2) - (zv1 * yv2);
        int yvn = (zv1 * xv2) - (xv1 * zv2);
        int zvn = (xv1 * yv2) - (yv1 * xv2);
        
        std::vector<int> vn{xvn, yvn, zvn};

        float a = vn[0];
        float b = vn[1];
        float c = vn[2];
        float d = -(a*x1 + b*y1 + c*z1);

		// Now, let's itterate through all of the points in cloud and measure each point's distance to the plane.
		// If that distance is less than or equal to our distanceTol, then group that point as an inlier
		for (int index = 0; index < cloud -> points.size(); index++)
		{
			if (inliers.count(index) > 0)  // checking to see if the element (index) exists in the inliers or not
				continue;
			
			pcl::PointXYZ point = cloud -> points[index]; // if the point isn't part of our model (the plane), then calculate the distance of the point from the plane and see if it is in the threshold or not
			float x4 = point.x;  // get the x member from point
			float y4 = point.y;  // get the y member from point
			float z4 = point.z;  // get the z member from point

			// now figure out the distance of the point from the plane.  Remember to do "float absolute" value (fabs) so there is no rounding and wierd results
			float dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

			// If the distance to the plane is less than or equal to our distanceTol number, include it in our inliers
			if (dist <= distanceTol)
				inliers.insert(index);
		}

		// Now check to see if our new list of inliers is greater than our best list of inliers to date.  Note: inlierResult starts out as zero.
		if (inliers.size() > inliersResult.size())  
		{
			inliersResult = inliers;
		}

	}
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.4);  // hyperparameters

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


	// Render 3D point cloud with inliers and outliers
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
