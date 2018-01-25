#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPLYFile("Test.ply", *showPoints);

	pcl::visualization::CloudViewer viewer("Clouds");

	viewer.showCloud(showPoints);

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

	kdtree.setInputCloud(showPoints);

	//找最近20個點

	int K = 20;

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	//假設我們想找距離原點最近的20個點

	pcl::PointXYZRGB searchPoint;

	searchPoint.x = 0.0;
	searchPoint.y = 0.0;
	searchPoint.z = 0.0;
	searchPoint.r = 0.0;
	searchPoint.g = 0.0;
	searchPoint.b = 0.0;

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << showPoints->points[pointIdxNKNSearch[i]].x
			<< " " << showPoints->points[pointIdxNKNSearch[i]].y
			<< " " << showPoints->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}



	//以半徑當作搜索條件
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 0.5; //0.5公尺附近

	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;


	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << showPoints->points[pointIdxRadiusSearch[i]].x
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