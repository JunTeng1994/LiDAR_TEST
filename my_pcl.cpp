#include "my_pcl.h"

PointCloud::Ptr laserScan2PointCloud(LaserScan& laserScan){
	float d = 0.0;
	PointCloud::Ptr cloud(new PointCloud);
	for (int i = 1; i < laserScan.ranges.size() - 1; i++){
		d = laserScan.ranges[i].getRho();
		PointT p;
		p.z = 0.0;
		p.x = d * cos(i * laserScan.angle_increment + laserScan.angle_min);
		p.y = d * sin(i * laserScan.angle_increment + laserScan.angle_min);

		p.r = 255;
		p.g = 0;
		p.b = 0;

		cloud->push_back(p);
	}
	return cloud;
}

/*
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, PointCloud::Ptr sweep, Eigen::Matrix4f transform) {
	PointCloud::Ptr output(new PointCloud());
	pcl::transformPointCloud(*original, *output, transform);
	*output += *sweep;

	static pcl::VoxelGrid<PointT> voxel;
	voxel.setLeafSize(0.1, 0.1, 0.1);
	voxel.setInputCloud(output);
	PointCloud::Ptr tmp(new PointCloud());
	voxel.filter(*tmp);

	return tmp;
//	return output;
}
*/

PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, PointCloud::Ptr sweep, Eigen::Matrix4f transform) {

	Eigen::Matrix4f transformTemp = Eigen::Matrix4f::Identity();
	transformTemp(2, 3) = -0.01;

	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(original);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.0, 0);
	PointCloud::Ptr passFiltered(new PointCloud());
	pass.filter(*passFiltered);

	PointCloud::Ptr output(new PointCloud());
	pcl::transformPointCloud(*passFiltered, *output, transformTemp);
	*output += *sweep;

	static pcl::VoxelGrid<PointT> voxel;
	voxel.setLeafSize(0.02, 0.02, 0.02);
	voxel.setInputCloud(output);
	PointCloud::Ptr tmp(new PointCloud());
	voxel.filter(*tmp);

	return tmp;
//	return output;
}

PointCloud::Ptr joinPointCloud1(PointCloud::Ptr original, PointCloud::Ptr newCloud, Eigen::Matrix4f transform) {
	
	static Eigen::Matrix4f transformTemp = Eigen::Matrix4f::Identity();
//	std::cout << transformTemp(2, 3) << std::endl;
	PointCloud::Ptr output(new PointCloud());
	pcl::transformPointCloud(*newCloud, *output, transformTemp);
	*output += *original;

//	std::cout << "output : " << output->points.size() << std::endl;

	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(output);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(transformTemp(2, 3) - 1.0, transformTemp(2, 3));
	PointCloud::Ptr passFiltered(new PointCloud());
	pass.filter(*passFiltered);
//	std::cout << "passFiltered : " << passFiltered->points.size() << std::endl;


	static pcl::VoxelGrid<PointT> voxel;
	voxel.setLeafSize(0.02, 0.02, 0.02);
	voxel.setInputCloud(passFiltered);
	PointCloud::Ptr tmp(new PointCloud());
	voxel.filter(*tmp);
	
	transformTemp(2, 3) += 0.01;

	return tmp;
}