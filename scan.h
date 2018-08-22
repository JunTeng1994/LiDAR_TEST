#pragma once
#include "common_header.h"
#include "scan.h"
#include "MedianFilter.h"
#include "Point.h"
#include "MobileAverage.h"
#include "angles.h"

struct LaserScan
{
public:
	struct Header {
		uint32_t sea;
		long double stamp;
		std::string frame_id;
	}header;
	float angle_min;
	float angle_max;
	float angle_increment;
	float time_increment;
	float scan_time;
	float range_min;
	float range_max;
	std::vector<Point> ranges;
	std::vector<float> intensities;

	LaserScan();
	LaserScan(const LaserScan &laserScan);
	LaserScan& operator= (const LaserScan& laserScan);
	void medianFilter(unsigned int windowSize = 15);
	void mobileAverage(unsigned int windowSize = 15);

	Point &operator[](int index){
		return ranges[index];
	}

	const Point &operator[](int index) const {
		return ranges[index];
	}

	unsigned int size() const {
		return ranges.size();
	}

	void push_back(Point p){
		ranges.push_back(p);
	}
	
	std::vector<Point>::const_iterator begin(){
		return ranges.begin();
	}

	std::vector<Point>::const_iterator end(){
		return ranges.end();
	}

	friend std::ostream &operator<< (std::ostream &os, const LaserScan &laserScan);


};
