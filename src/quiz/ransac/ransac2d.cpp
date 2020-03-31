/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
        while(maxIterations--)
            {
                // pick three points to define plane
                std::unordered_set<int> inliers;
                while (inliers.size() < 3)
                    inliers.insert(rand()%(cloud->points.size()));

                // generate plane vectors v1 and v2
                auto idx = inliers.begin();
                pcl::PointXYZ p1 = cloud->points[*idx];
                idx++;
                pcl::PointXYZ p2 = cloud->points[*idx];
                idx++;
                pcl::PointXYZ p3 = cloud->points[*idx];
                // idx++;
                
                float v1x = p2.x - p1.x;
                float v1y = p2.y - p1.y;
                float v1z = p2.z - p1.z;

                float v2x = p3.x - p1.x;
                float v2y = p3.y - p1.y;
                float v2z = p3.z - p1.z;

                // find normal vector n (cross product)
                float nx = v1y*v2z - v2y*v1z;
                float ny = v1z*v2x - v2z*v1x;
                float nz = v1x*v2y - v2x*v1y;

                // back to nomenclature of the script
                float A = nx;
                float B = ny;
                float C = nz;
                float D = -(A*p1.x + B*p1.y + C*p1.z);
                
                // Measure distance between every point and fitted plane
                // If distance is smaller than threshold count it as inlier
                for(int index=0; index<cloud->points.size(); index++)
                    {
                        // ignore the 3 points we used to generate the line
                        if(inliers.count(index)>0)
                            continue;

                        pcl::PointXYZ point = cloud->points[index];
                        float x3 = point.x;
                        float y3 = point.y;
                        float z3 = point.z;

                        float d = fabs(A*x3 + B*y3 + C*z3 + D) / sqrt(A*A + B*B + C*C);

                        if(d<=distanceTol)
                            inliers.insert(index);
                    }

                // keep the one with most points within distanceTol
                if(inliers.size() > inliersResult.size())
                    {
                        inliersResult = inliers;
                    }
        }

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 1000, 0.3);

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
