#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>



int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPointsSrc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("points.pcd", *showPointsSrc);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr showPointsTar(new pcl::PointCloud<pcl::PointXYZRGB>);

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	float theta = M_PI / 18; // 10 degree, The angle of rotation in radians
	transform(0, 0) = cos(theta);
	transform(0, 1) = -sin(theta);
	transform(1, 0) = sin(theta);
	transform(1, 1) = cos(theta);

	transform(0, 3) = 0.5; //x
	transform(1, 3) = 0.5; //y
	transform(2, 3) = 0.5; //z

	std::cout << "transform: " << std::endl << transform << std::endl;

	pcl::transformPointCloud(*showPointsSrc, *showPointsTar, transform);

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

	icp.setInputCloud(showPointsSrc);
	icp.setInputTarget(showPointsTar);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << std::endl <<  " score: " << icp.getFitnessScore() << std::endl;
	std::cout << "icp get transformation: " << std::endl << icp.getFinalTransformation() << std::endl;
	
	return 0;
}