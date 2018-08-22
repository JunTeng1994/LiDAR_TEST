#pragma once

#define NOMINMAX
#include "LMS400.h"

#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


#include <boost\timer.hpp>

#include <vector>
#include <queue>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZ Point3D;
typedef pcl::PointCloud<PointT> PointCloud;

class LaserScan2PCL {
public:
	LaserScan2PCL();
	~LaserScan2PCL();
private:
	LMS400 *m_lms;
	// 参数配置类
	ParameterReader *m_pd;
	// 激光数据转换为点云
	PointCloud::Ptr laserScan2PointCloud(LaserScan& laserScan);
	PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, PointCloud::Ptr newCloud, Eigen::Matrix4f transform);
	PointCloud::Ptr joinPointCloud1(PointCloud::Ptr original, PointCloud::Ptr newCloud, Eigen::Matrix4f transform);
	// 提取4个关键点
	std::vector<Point3D> extractCornerFromLaserScan(LaserScan& laserScan);
	//通过4个顶点画长方形
	PointCloud::Ptr drawRectanglePointCloud(std::vector<Point3D> &vertex);
	// 画长方形的4个顶点
	PointCloud::Ptr drawRectangleVertexPointCloud(std::vector<Point3D> &vertex);
	// 两点画直线 1米内画100个点  颜色为 1--->r; 2--->g; 3--->b; 4--->白色
	PointCloud::Ptr drawLinePointCloud(Point3D &startPoint,Point3D &endPoint, int color = 0, int numPerMeter = 100);
	PointCloud::Ptr drawVertexPointCloud(std::vector<Point3D>& vertex);
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
	std::vector<std::vector<Point3D> > getCornerPoints();
	void final(std::vector<Point3D>& vertex);
	void spin();
	void calculateScanFrequency(LaserScan& laserScan);
	Point3D addZVelocity(Point3D& point, long double durTime);

	Eigen::Matrix4f m_transform;
	// 可视化工具
	boost::shared_ptr<pcl::visualization::CloudViewer> m_viewer;

	int m_startPointIndex; // 开始点下标
	int m_endPointIndex;   // 结束点下标
	double m_curThreshold;

	std::vector<std::vector<Point> > m_lineCornerPoints;
	std::vector<Point3D> m_laserScanVertex;
	bool m_flag;

	//两次扫描之间z轴移动位移
	long double m_zMoveDistance;

	long double m_elapseTime;

	//4关键点存在这里
	std::vector<std::vector<Point3D> > m_cornerPoints; 
	//物体进入标志
	bool m_startProcessFlag;   
	//物体离开标志
	bool m_finishProcessFlag;

	bool m_updateFlag;

	bool m_clearFlag;
	//第一帧激光数据
	bool m_firstLaserScanFlag;

	//z轴的移动速度
	double m_zSpeed;

	//距离差的最小值
	double m_diffMinThreshold;
	
	bool m_canGetLaserScanFlag;


	PointCloud::Ptr drawObjectFromCornerPoints(std::vector<std::vector<Point3D> > cornerPoints);


	double m_startTime;
	double m_endTime;
	long double m_getLaserTime;
	
#define ArrayNum 10
	int m_findFlagArray[ArrayNum];
};

