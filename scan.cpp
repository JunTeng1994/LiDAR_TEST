#include "scan.h"

LaserScan::LaserScan():
ranges(700, Point::Polar(0.0,0.0))
{
	intensities.resize(700);
}

LaserScan::LaserScan(const LaserScan &laserScan){
	this->header.sea = laserScan.header.sea;
	this->header.stamp = laserScan.header.stamp;
	this->header.frame_id = laserScan.header.frame_id;

	this->angle_min = laserScan.angle_min;
	this->angle_max = laserScan.angle_max;
	this->angle_increment = laserScan.angle_increment;
	this->time_increment = laserScan.time_increment;
	this->scan_time = laserScan.scan_time;
	this->range_min = laserScan.range_min;
	this->range_max = laserScan.range_max;
	this->ranges.assign(laserScan.ranges.begin(), laserScan.ranges.end());
	this->intensities.assign(laserScan.intensities.begin(), laserScan.intensities.end());
}
LaserScan& LaserScan::operator= (const LaserScan& laserScan){
	this->header.sea = laserScan.header.sea;
	this->header.stamp = laserScan.header.stamp;
	this->header.frame_id = laserScan.header.frame_id;

	this->angle_min = laserScan.angle_min;
	this->angle_max = laserScan.angle_max;
	this->angle_increment = laserScan.angle_increment;
	this->time_increment = laserScan.time_increment;
	this->scan_time = laserScan.scan_time;
	this->range_min = laserScan.range_min;
	this->range_max = laserScan.range_max;
	this->ranges.assign(laserScan.ranges.begin(), laserScan.ranges.end());
	this->intensities.assign(laserScan.intensities.begin(), laserScan.intensities.end());
	return *this;
}

void LaserScan::medianFilter(unsigned int windowSize){
	const unsigned int size = ranges.size();
	//const unsigned int size = 5;
	//double rhos[size], rhosmedian[size];
	double *rhos = new double[size];
	double *rhosmedian = new double[size];


	for (unsigned int i = 0; i < size; i++)
		rhos[i] = ranges[i].getRho();

	doMedianFilter(rhos, rhosmedian, size, windowSize);

	for (unsigned int i = 0; i < size; i++)
		ranges[i].setRho(rhosmedian[i]);

	delete[] rhos;
	delete[] rhosmedian;
}

void LaserScan::mobileAverage(unsigned int windowSize){
	unsigned int size = ranges.size();
	//double source[size], targetX[size], targetY[size];
	double *source = new double[size];
	double *targetX = new double[size];
	double *targetY = new double[size];

	for (unsigned int i = 0; i < size; i++)
		source[i] = ranges[i].getX();

	doMobileAverage(source, targetX, size, windowSize);

	for (unsigned int i = 0; i < size; i++)
		source[i] = ranges[i].getY();

	doMobileAverage(source, targetY, size, windowSize);

	for (unsigned int i = 0; i < size; i++)
		ranges[i] = Point::Cartesian(targetX[i], targetY[i]);

	delete[] source;
	delete[] targetX;
	delete[] targetY;
}


std::ostream &operator<< (std::ostream &os, const LaserScan& laserScan){
	os << "angle_min -------> " << angles::to_degrees(laserScan.angle_min) << std::endl;
	os << "angle_max -------> " << angles::to_degrees(laserScan.angle_max) << std::endl;
	os << "angle_increment -------> " << angles::to_degrees(laserScan.angle_increment) << std::endl;
	os << "time_increment -------> " << laserScan.time_increment << std::endl;
	os << "scan_time -------> " << laserScan.scan_time << std::endl;
	os << "range_min -------> " << laserScan.range_min << std::endl;
	os << "range_max -------> " << laserScan.range_max << std::endl;

	/*
	unsigned int i = laserScan.ranges.size();
	os << "{";
	for (auto it = laserScan.ranges.begin(); it < laserScan.ranges.end(); it++){
		os << (*it);
		if (--i > 0)
			os << ", ";
	}
	*/

	return os << "}";
}

