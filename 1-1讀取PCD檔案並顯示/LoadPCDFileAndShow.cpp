#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("points.pcd", *showPoints);

	pcl::visualization::CloudViewer viewer("Clouds");

	viewer.showCloud(showPoints);

	while (!viewer.wasStopped()) 
	{
		//�]���o��O�Mvisualization�������P�B�]�A�ҥH�i�H�b��������@�ǳB�z
	}
	
	return 0;
}