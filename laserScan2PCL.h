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
	// ����������
	ParameterReader *m_pd;
	// ��������ת��Ϊ����
	PointCloud::Ptr laserScan2PointCloud(LaserScan& laserScan);
	PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, PointCloud::Ptr newCloud, Eigen::Matrix4f transform);
	PointCloud::Ptr joinPointCloud1(PointCloud::Ptr original, PointCloud::Ptr newCloud, Eigen::Matrix4f transform);
	// ��ȡ4���ؼ���
	std::vector<Point3D> extractCornerFromLaserScan(LaserScan& laserScan);
	//ͨ��4�����㻭������
	PointCloud::Ptr drawRectanglePointCloud(std::vector<Point3D> &vertex);
	// �������ε�4������
	PointCloud::Ptr drawRectangleVertexPointCloud(std::vector<Point3D> &vertex);
	// ���㻭ֱ�� 1���ڻ�100����  ��ɫΪ 1--->r; 2--->g; 3--->b; 4--->��ɫ
	PointCloud::Ptr drawLinePointCloud(Point3D &startPoint,Point3D &endPoint, int color = 0, int numPerMeter = 100);
	PointCloud::Ptr drawVertexPointCloud(std::vector<Point3D>& vertex);
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
	std::vector<std::vector<Point3D> > getCornerPoints();
	void final(std::vector<Point3D>& vertex);
	void spin();
	void calculateScanFrequency(LaserScan& laserScan);
	Point3D addZVelocity(Point3D& point, long double durTime);

	Eigen::Matrix4f m_transform;
	// ���ӻ�����
	boost::shared_ptr<pcl::visualization::CloudViewer> m_viewer;

	int m_startPointIndex; // ��ʼ���±�
	int m_endPointIndex;   // �������±�
	double m_curThreshold;

	std::vector<std::vector<Point> > m_lineCornerPoints;
	std::vector<Point3D> m_laserScanVertex;
	bool m_flag;

	//����ɨ��֮��z���ƶ�λ��
	long double m_zMoveDistance;

	long double m_elapseTime;

	//4�ؼ����������
	std::vector<std::vector<Point3D> > m_cornerPoints; 
	//��������־
	bool m_startProcessFlag;   
	//�����뿪��־
	bool m_finishProcessFlag;

	bool m_updateFlag;

	bool m_clearFlag;
	//��һ֡��������
	bool m_firstLaserScanFlag;

	//z����ƶ��ٶ�
	double m_zSpeed;

	//��������Сֵ
	double m_diffMinThreshold;
	
	bool m_canGetLaserScanFlag;


	PointCloud::Ptr drawObjectFromCornerPoints(std::vector<std::vector<Point3D> > cornerPoints);


	double m_startTime;
	double m_endTime;
	long double m_getLaserTime;
	
#define ArrayNum 10
	int m_findFlagArray[ArrayNum];
};

