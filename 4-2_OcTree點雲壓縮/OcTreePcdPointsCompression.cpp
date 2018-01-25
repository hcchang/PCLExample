#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/compression/octree_pointcloud_compression.h>


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("points.pcd", *showPoints);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPointsOut(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
	bool showStatistics = true;

	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>::Ptr PointCloudEncoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfile, showStatistics));

	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>::Ptr PointCloudDecoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>());

	std::stringstream compressedData;

	PointCloudEncoder->encodePointCloud(showPoints, compressedData);

	PointCloudDecoder->decodePointCloud(compressedData, showPointsOut);

	pcl::visualization::CloudViewer viewer("Clouds");

	viewer.showCloud(showPointsOut);

	while (!viewer.wasStopped()) 
	{
		//�]���o��O�Mvisualization�������P�B�]�A�ҥH�i�H�b��������@�ǳB�z
	}
	
	return 0;
}