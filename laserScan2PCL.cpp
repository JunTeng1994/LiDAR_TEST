#include "laserScan2PCL.h"


LaserScan2PCL::LaserScan2PCL() :
//m_startPointIndex(1),
//m_endPointIndex(699),
//m_zSpeed(0.01),
m_flag(false),
m_clearFlag(true),
m_startProcessFlag(false),
m_firstLaserScanFlag(true),
m_finishProcessFlag(true),
m_canGetLaserScanFlag(false),
m_updateFlag(false),
m_zMoveDistance(0.0),
m_startTime(0.0),
m_getLaserTime(0.0),
m_endTime(0.0)
{
	m_lms = new LMS400();

	for (int i = 0; i < ArrayNum; i++){
		m_findFlagArray[i] = 0;
	}

	/////////////配置参数///////////////////////////////////////
	m_pd = new ParameterReader("laserScan2PCL.txt");
	m_startPointIndex  = atoi(m_pd->getData("startPointIndex").c_str());
	m_endPointIndex    = atoi(m_pd->getData("endPointIndex").c_str());
	m_zSpeed           = atof(m_pd->getData("zSpeed").c_str());
	m_diffMinThreshold = atof(m_pd->getData("diffMinThreshold").c_str());
	/////////////end配置参数///////////////////////////////////////

	m_transform = Eigen::Matrix4f::Identity();
	m_viewer.reset(new pcl::visualization::CloudViewer("PCL Windows"));
	m_viewer->registerKeyboardCallback(&LaserScan2PCL::keyboardEventOccurred, *this);
	spin();
}

LaserScan2PCL::~LaserScan2PCL(){
	delete m_pd;
	delete m_lms;
}

PointCloud::Ptr LaserScan2PCL::laserScan2PointCloud(LaserScan& laserScan){
	PointCloud::Ptr cloud(new PointCloud());
	for (int i = m_startPointIndex; i < m_endPointIndex; i++){
		PointT p;
		p.z = 0.0;
		p.x = laserScan[i].getX();
		p.y = laserScan[i].getY();
		if (i < m_startPointIndex + 5){
			p.r = 255;
			p.g = 255;
			p.b = 255;
		}
		else if (i > m_endPointIndex - 5){
			p.r = 0;
			p.g = 0;
			p.b = 255;
		}
		else {
			p.r = 255;
			p.g = 0;
			p.b = 0;
		}

		cloud->push_back(p);
	}
	return cloud;
}


PointCloud::Ptr LaserScan2PCL::joinPointCloud(PointCloud::Ptr original, PointCloud::Ptr newCloud, Eigen::Matrix4f transform){
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
	*output += *newCloud;

	static pcl::VoxelGrid<PointT> voxel;
	voxel.setLeafSize(0.02, 0.02, 0.02);
	voxel.setInputCloud(output);
	PointCloud::Ptr tmp(new PointCloud());
	voxel.filter(*tmp);

	return tmp;
}

PointCloud::Ptr LaserScan2PCL::joinPointCloud1(PointCloud::Ptr original, PointCloud::Ptr newCloud, Eigen::Matrix4f transform){
	static Eigen::Matrix4f transformTemp = Eigen::Matrix4f::Identity();
	PointCloud::Ptr output(new PointCloud());
	pcl::transformPointCloud(*newCloud, *output, transformTemp);
	*output += *original;


	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(output);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(transformTemp(2, 3) - 1.0, transformTemp(2, 3));
	PointCloud::Ptr passFiltered(new PointCloud());
	pass.filter(*passFiltered);


	static pcl::VoxelGrid<PointT> voxel;
	voxel.setLeafSize(0.02, 0.02, 0.02);
	voxel.setInputCloud(passFiltered);
	PointCloud::Ptr tmp(new PointCloud());
	voxel.filter(*tmp);
	
	transformTemp(2, 3) += 0.01;

	return tmp;
}

void LaserScan2PCL::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing){
//	std::cout << event.getKeySym() << std::endl;
	if (event.getKeySym() == "k" && event.keyDown()){
	}
	else if (event.getKeySym() == "j" && event.keyDown()){
	}
	if (event.getKeySym() == "s" && event.keyDown()){
	}
	else if (event.getKeySym() == "w" && event.keyDown()){
	}
	else if (event.getKeySym() == "Up" && event.keyDown()){
	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
	}
	else if (event.getKeySym() == "t" && event.keyDown()){
		m_flag = !m_flag;
		if (m_flag){
			std::cout << "enable flag" << std::endl;
		}
		else {
			std::cout << "disenable flag" << std::endl;
		}
	}
	else if (event.getKeySym() == "i" && event.keyDown()){
		m_clearFlag = true;
	}
	else if (event.getKeySym() == "space" && event.keyDown()){
		m_clearFlag = false;
	}
}

//void LaserScan2PCL::spin(){
//
//	LaserScan *laserScan = new LaserScan();
//	*laserScan = m_lms->getLaserScan();
//	std::vector<Point3D> laserScanVertex;
//	PointCloud::Ptr pointCloudShow(new PointCloud());
//	while (1){
//		if (m_flag){
//			if (m_clearFlag){
//				for (int i = 0; i < laserScan->size(); i++){
//					laserScan->ranges[i].setTheta(laserScan->angle_increment * i + laserScan->angle_min);
//					laserScan->ranges[i].setY(2.289);
//				}
//			}
//			else {
//				*laserScan = m_lms->getLaserScan();
//				laserScan->medianFilter(30); // 滤波处理
//			}
//
//			pointCloudShow = laserScan2PointCloud(*laserScan);
//			laserScanVertex = extractCornerFromLaserScan(*laserScan);
//			*pointCloudShow += *drawObjectFromCornerPoints(m_cornerPoints);
//			if (m_canGetLaserScanFlag){
//				m_canGetLaserScanFlag = false;
//			}
//			else {
//
//			}
//			m_viewer->showCloud(pointCloudShow);
//		}
//		else {
//			*laserScan = m_lms->getLaserScan();
//			laserScan->medianFilter(30); // 滤波处理
//			pointCloudShow = laserScan2PointCloud(*laserScan);
//			m_viewer->showCloud(pointCloudShow);
//		}
//
//
//	}
//	delete laserScan;
//}

void LaserScan2PCL::spin(){
	LaserScan *laserScan = new LaserScan();
	PointCloud::Ptr pointCloudShow(new PointCloud());
	std::vector<Point3D> laserScanVertex;
	int showCnt = 0;
	while (1){
		*laserScan = m_lms->getLaserScan();
		m_getLaserTime = laserScan->header.stamp;
//		laserScan->medianFilter(); // 滤波处理  特别耗时
		calculateScanFrequency(*laserScan);
		laserScanVertex = extractCornerFromLaserScan(*laserScan);
		final(laserScanVertex);

		pointCloudShow = laserScan2PointCloud(*laserScan);
		m_viewer->showCloud(pointCloudShow);

	/*	if (m_updateFlag){
			m_updateFlag = false;
			for (int i = 0; i < m_cornerPoints.size(); i++){
				*pointCloudShow += *drawVertexPointCloud(m_cornerPoints[i]);
			}

			*pointCloudShow += *drawVertexPointCloud(laserScanVertex);
			m_viewer->showCloud(pointCloudShow);
		}*/
	}
}

void LaserScan2PCL::calculateScanFrequency(LaserScan& laserScan){
	static long double lastTime = 0;
	static long double currTime = 0;
	static int ccnt = 0;
	static long double sum = 0;
		if (m_firstLaserScanFlag){
			m_firstLaserScanFlag = false;
			lastTime = laserScan.header.stamp;
		}
		else {
			currTime = laserScan.header.stamp;
			m_elapseTime = (currTime - lastTime);
			lastTime = currTime;
			sum += m_elapseTime;
			//if (ccnt < 100){
			//	ccnt++;
			//}
			//else {
			//	std::cout << "Average scan time 100 times: " << (1.0 / (sum / ccnt)) * 1000<< " Hz" << std::endl;
			//	ccnt = 0;
			//	sum = 0.0;
			//}
			if (++ccnt > 99){
				std::cout << "Average scan time 100 times: " << (1.0 / (sum / ccnt))  * 1000 << " Hz" << std::endl;
				ccnt = 0;
				sum = 0.0;
			}
		}
}

std::vector<Point3D> LaserScan2PCL::extractCornerFromLaserScan(LaserScan& laserScan){
	std::vector<Point3D> output;
	bool findLeftPointFlag = false;
	bool findRightPointFlag = false;
	Point3D leftPoint3D;
	Point3D rightPoint3D;
	int leftIndex = m_endPointIndex;
	double diff_max = -1000.0;
	double diff_min = 1000.0;
	std::vector<double> diff(laserScan.size());
	double diff_temp;
	int diff_max_index = m_endPointIndex;
	int diff_min_index = m_startPointIndex;
	for (int i = m_startPointIndex; i < m_endPointIndex - 1; i++){
		diff_temp = laserScan[i].getY() - laserScan[i + 1].getY();
		if (diff_temp >= diff_max){
			diff_max = diff_temp;
			diff_max_index = i;
		}
		else if (diff_temp <= diff_min)
		{
			diff_min = diff_temp;
			diff_min_index = i;
		}
		diff[i] = diff_temp;
	}
	if (diff_max < m_diffMinThreshold || diff_min > -m_diffMinThreshold){
		diff_max_index = m_endPointIndex;
		diff_min_index = m_startPointIndex;
	}

	for (int i = diff_max_index + 1; i < diff_min_index - 1; i++){
		if (abs(diff[i]) < 0.005){
			leftPoint3D.x = laserScan[i].getX();
			leftPoint3D.y = laserScan[i].getY();
			leftPoint3D.z = 0.0;
			findLeftPointFlag = true;
			leftIndex = i;
			output.push_back(leftPoint3D);
			break;
		}
	}
	for (int i = diff_min_index - 1; i > leftIndex; i--){
		if (abs(diff[i]) < 0.005){
			rightPoint3D.x = laserScan[i].getX();
			rightPoint3D.y = laserScan[i].getY();
			rightPoint3D.z = 0.0;
			findRightPointFlag = true;
			output.push_back(rightPoint3D);
			break;
		}
	}
	if (findLeftPointFlag && findRightPointFlag){
//		std::cout << "find object" << std::endl;
		std::vector<double> distance;
		double max_distance = 0.0;
		for (int i = m_startPointIndex; i < m_endPointIndex; i++){
			distance.push_back(laserScan[i].getY());
		}
		std::sort(distance.begin(), distance.end());
		max_distance = std::accumulate(distance.end() - 20, distance.end() - 10, 0.0) / 10.0;
		rightPoint3D.y = max_distance;
		output.push_back(rightPoint3D);
		leftPoint3D.y = max_distance;
		output.push_back(leftPoint3D);
	}
	return output;
}



PointCloud::Ptr LaserScan2PCL::drawRectanglePointCloud(std::vector<Point3D> &vertex){
	PointCloud::Ptr output(new PointCloud());
	Point3D startPoint;
	Point3D endPoint;
	//startPoint = vertex[0];
	//endPoint = vertex[1];
	//*output += *drawLinePointCloud(startPoint, endPoint, 1);
	for (int i = 0; i < vertex.size(); i++){
		if (i < vertex.size() - 1){
			startPoint = vertex[i];
			endPoint = vertex[i + 1];
		}
		else {
			startPoint = vertex[i];
			endPoint = vertex[0];
		}
		*output += *drawLinePointCloud(startPoint, endPoint, i + 1);
	}
	return output;
}

PointCloud::Ptr LaserScan2PCL::drawLinePointCloud(Point3D &startPoint, Point3D &endPoint, int color, int numPerMeter){
	PointCloud::Ptr output(new PointCloud());
	PointT p;
	double x0 = startPoint.x;
	double y0 = startPoint.y;
	double z0 = startPoint.z;
	double x1 = endPoint.x;
	double y1 = endPoint.y;
	double z1 = endPoint.z;
	double distance = sqrt(pow((x1 - x0), 2.0) + pow((y1 - y0), 2.0) + pow((z1 - z0), 2.0));
//	std::cout << "distance ---> " << distance << std::endl;
	double xDelta = (x1 - x0) / (distance * numPerMeter);
	double yDelta = (y1 - y0) / (distance * numPerMeter);
	double zDelta = (z1 - z0) / (distance * numPerMeter);
	for (int i = 0; i < distance * numPerMeter; i++){
		p.x = x0 + i * xDelta;
		p.y = y0 + i * yDelta;
		p.z = z0 + i * zDelta;
		switch (color)  {
		case 1 :
			p.r = 255;
			p.g = 0;
			p.b = 0;
			break;
		case 2 :
			p.r = 0;
			p.g = 255;
			p.b = 0;
			break;
		case 3 :
			p.r = 0 ;
			p.g = 0;
			p.b = 255;
			break;
		case 4 :
			p.r = 255 ;
			p.g = 255;
			p.b = 255;
			break;
		default:
			break;
		}
		if ((x0 - p.x) * (x1 - p.x) <= 0 && (y0 - p.y) * (y1 - p.y) <= 0 && (z0 - p.z) * (z1 - p.z) <= 0){
			output->push_back(p);
		}
	}
//	std::cout << "endPoint.x--> " << endPoint.x << " ,endPoint.y--> " << endPoint.y << " ,endPoint.z--> " << endPoint.z << std::endl;
//	std::cout << "p.x--> " << p.x << " ,p.y--> " << p.y << " ,p.z--> " << p.z << std::endl;
	return output;
}

PointCloud::Ptr LaserScan2PCL::drawRectangleVertexPointCloud(std::vector<Point3D> &vertex){
	PointCloud::Ptr output(new PointCloud());
	PointT point;
	for (int i = 0; i < vertex.size(); i++){
		point.x = vertex[i].x;
		point.y = vertex[i].y;
		point.z = vertex[i].z;
		switch (i) {
		case 0:
			point.r = 255;
			point.g = 0;
			point.b = 0;
			break;
		case 1:
			point.r = 0;
			point.g = 255;
			point.b = 0;
			break;
		case 2:
			point.r = 0;
			point.g = 0;
			point.b = 255;
			break;
		case 3:
			point.r = 255;
			point.g = 255;
			point.b = 255;
			break;
		default:
			break;
		}
		output->push_back(point);
	}
	return output;
}



std::vector<std::vector<Point3D> > LaserScan2PCL::getCornerPoints(){
	spin();
	if (m_canGetLaserScanFlag){
		m_canGetLaserScanFlag = false;
		return m_cornerPoints;
	}
	else {
		return std::vector<std::vector<Point3D> >{};
	}
}

PointCloud::Ptr LaserScan2PCL::drawObjectFromCornerPoints(std::vector<std::vector<Point3D> > cornerPoints){
	PointCloud::Ptr output(new PointCloud());
	for (int i = 0; i < cornerPoints.size(); i++){
		*output += *drawRectanglePointCloud(cornerPoints[i]);
	}
	return output;
}

PointCloud::Ptr LaserScan2PCL::drawVertexPointCloud(std::vector<Point3D>& vertex){
	PointCloud::Ptr output(new PointCloud());
	if (vertex.size() == 0){
		return output;
	}
	else
	{
		PointT p;
		for (int i = 0; i < vertex.size(); i++){
			p.x = vertex[i].x;
			p.y = vertex[i].y;
			p.z = vertex[i].z;
			p.r = 255;
			p.g = 255;
			p.b = 255;
			output->push_back(p);
		}
	}
	return output;
}

void LaserScan2PCL::final(std::vector<Point3D>& vertex){
	//if (vertex.size() == 0){
	//	if (m_startProcessFlag){
	//		m_finishProcessFlag = true;
	//		m_startProcessFlag = false;
	//		std::cout << "finish" << std::endl;
	//	}
	//}
	//else
	//{
	//	if (!m_startProcessFlag){
	//		m_startProcessFlag = true;
	//		m_finishProcessFlag = false;
	//		std::cout << "start" << std::endl;
	//	}
	//}

	for (int i = ArrayNum - 1; i > 0; i--){
		m_findFlagArray[i] = m_findFlagArray[i - 1];
	}

	if (vertex.size() == 0){
		m_findFlagArray[0] = 0;
	}
	else {
		m_findFlagArray[0] = 1;
	}

	if (std::accumulate(m_findFlagArray, m_findFlagArray + 5, 0) == 0){
		if (m_startProcessFlag){
			m_finishProcessFlag = true;
			m_startProcessFlag = false;
			m_updateFlag = true;
			std::cout << "finish" << std::endl;
		}
	}
	if (std::accumulate(m_findFlagArray, m_findFlagArray + 5, 0) == 5){
		if (!m_startProcessFlag){
			m_startProcessFlag = true;
			m_finishProcessFlag = false;
			m_startTime = m_getLaserTime;
			m_cornerPoints.clear();
			std::cout << "start" << std::endl;
		}
		else if (!m_finishProcessFlag){
			m_zMoveDistance = m_elapseTime * m_zSpeed / 1000.0;
			for (int i = 0; i < m_cornerPoints.size() - 1; i++){
				for (int j = 0; j < m_cornerPoints[i].size(); j++){
					m_cornerPoints[i][j].z += m_zMoveDistance;
				}
			}
		}
		m_cornerPoints.push_back(vertex);
	}
}

Point3D LaserScan2PCL::addZVelocity(Point3D& point, long double durTime){
	Point3D res(point);
	res.z += durTime * m_zSpeed;
	return res;
}

