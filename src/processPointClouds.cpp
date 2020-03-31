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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
              << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
  
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_ROI (new pcl::PointCloud<PointT> ());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_ROI);

    // remove the static points of the roof of the car
    std::vector<int>indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloud_ROI);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point: indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_ROI);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
  
    return cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

  for(int index: inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
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

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg; 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new  pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planar component from the cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneMS(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    std::unordered_set<int> inliersResult;
    // srand(time(NULL));

    // For max iterations
    while(maxIterations--)
        {
            // pick three points to define plane
            std::unordered_set<int> inliers;
            while (inliers.size() < 3)
                inliers.insert(rand()%(cloud->points.size()));

            // generate plane vectors v1 and v2
            auto idx = inliers.begin();
            PointT p1 = cloud->points[*idx];
            idx++;
            PointT p2 = cloud->points[*idx];
            idx++;
            PointT p3 = cloud->points[*idx];
            // idx++;
                
            const float v1x = p2.x - p1.x;
            const float v1y = p2.y - p1.y;
            const float v1z = p2.z - p1.z;

            const float v2x = p3.x - p1.x;
            const float v2y = p3.y - p1.y;
            const float v2z = p3.z - p1.z;

            // find normal vector n (cross product)
            const float A = v1y*v2z - v2y*v1z;  // nx
            const float B = v1z*v2x - v2z*v1x;  // ny
            const float C = v1x*v2y - v2x*v1y;  // nz
            const float D = -(A*p1.x + B*p1.y + C*p1.z);

            // Measure distance between every point and fitted plane
            // If distance is smaller than threshold count it as inlier
            const float denom = sqrt(A*A + B*B + C*C);
            for(int index=0; index<cloud->points.size(); index++)
                {
                    // ignore the 3 points we used to generate the line
                    if(inliers.count(index)>0)
                        continue;

                    PointT point = cloud->points[index];
                    float d = fabs(A*point.x + B*point.y + C*point.z + D) / denom;

                    if(d<=distanceThreshold)
                        inliers.insert(index);
                }

            // keep the one with most points within distanceTol
            if(inliers.size() > inliersResult.size())
                inliersResult = inliers;
        }
    
    if(inliersResult.size() == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // move indices from unordered_set to PointIndices 
    pcl::PointIndices::Ptr pp_inliers (new pcl::PointIndices);
    for(int index: inliersResult)
        pp_inliers->indices.push_back(index);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(pp_inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // cluster_indices is vector of indices here - each one defining a cluster
    for(int c=0; c<cluster_indices.size(); ++c)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT> ());   
            for(int index: cluster_indices[c].indices)
                cloudCluster->points.push_back(cloud->points[index]);

            // next 3 lines blindely copied from reference result - works without!
            cloudCluster->width  = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = 1;
            
            clusters.push_back(cloudCluster);
        }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


// \FIXME parameter cluster should be by reference - check!
template<typename PointT>
void ProcessPointClouds<PointT>::cluster_rec(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, typename ms::KdTree<PointT>* tree, float distanceTol)
{
    processed[index] = true;
    cluster.push_back(index);

    std::vector<int> nearest = tree->search(cloud->points[index], distanceTol);

    for(int id : nearest)
        {
            if(!processed[id])
                cluster_rec(id, cloud, cluster, processed, tree, distanceTol);
        }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringMS(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // generate KdTree using my own implementation
    ms::KdTree<PointT> tree;
    
    // insert all points into the tree
    // \FIXME we should sort and use the median
    for(int idx = 0; idx<cloud->size(); ++idx)
        tree.insert(cloud->points[idx], idx);
    
    // clustering
    std::vector<std::vector<int>> cluster_indices;
    std::vector<bool> processed(cloud->size(), false);   // bit vector to mark if a point was processed already

    for(int i=0; i<cloud->size(); ++i){
        if(!processed[i]){
            std::vector<int> cluster;
            cluster_rec(i, cloud, cluster, processed, &tree, clusterTolerance);
            cluster_indices.push_back(cluster);
        }
    }

    // cluster_indices is a vector of vectors each containing the indices for one cluster
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    for(int c=0; c<cluster_indices.size(); ++c)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT> ());   
            for(int index: cluster_indices[c])
                cloudCluster->points.push_back(cloud->points[index]);

            // only add clusters containing between minSize...maxSize points
            if( (cloudCluster->points.size()>=minSize) &&
                (cloudCluster->points.size()<=maxSize) ){
            
                // next 3 lines blindely copied from reference result - works without!
                cloudCluster->width  = cloudCluster->points.size();
                cloudCluster->height = 1;
                cloudCluster->is_dense = 1;
                
                clusters.push_back(cloudCluster);
            }
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
