#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("points.pcd", *showPoints);

	pcl::visualization::CloudViewer viewer("Clouds");

	viewer.showCloud(showPoints);

	float resolution = 0.03;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octreePoint(resolution);

	double minX = 0.0;
	double minY = 0.0;
	double minZ = 0.0;
	double maxX = 0.0;
	double maxY = 0.0;
	double maxZ = 0.0;

	octreePoint.setInputCloud(showPoints);
	octreePoint.addPointsFromInputCloud();
	octreePoint.getBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);

	float xLength = (float)abs(maxX - minX);
	float yLength = (float)abs(maxY - minY);
	float zLength = (float)abs(maxZ - minZ);

	float volume = xLength*yLength*zLength;

	int K = 10;

	pcl::PointXYZRGB searchPointKNN;

	searchPointKNN.x = 0.0;
	searchPointKNN.y = 0.0;
	searchPointKNN.z = 0.6;

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;


	if (octreePoint.nearestKSearch(searchPointKNN, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "nearestKSearch: " << showPoints->points[pointIdxNKNSearch[i]].x
			<< " " << showPoints->points[pointIdxNKNSearch[i]].y
			<< " " << showPoints->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}


	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 0.99;

	pcl::PointXYZRGB searchPoint;

	searchPoint.x = 0.0;
	searchPoint.y = 0.0;
	searchPoint.z = 0.6;

	if (octreePoint.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "radiusSearch: " << showPoints->points[pointIdxRadiusSearch[i]].x
			<< " " << showPoints->points[pointIdxRadiusSearch[i]].y
			<< " " << showPoints->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}


	while (!viewer.wasStopped()) 
	{
		//因為這邊是和visualization的視窗同步跑，所以可以在之中執行一些處理
	}
	
	return 0;
}