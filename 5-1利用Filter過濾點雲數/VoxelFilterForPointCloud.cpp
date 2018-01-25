#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>




int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("points.pcd", *showPoints);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPointsFilter(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::visualization::CloudViewer viewer("Clouds");


	float resolution = 0.03;

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;

	sor.setInputCloud(showPoints);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*showPointsFilter);


	viewer.showCloud(showPointsFilter);


	while (!viewer.wasStopped()) 
	{
		//�]���o��O�Mvisualization�������P�B�]�A�ҥH�i�H�b��������@�ǳB�z
	}
	
	return 0;
}