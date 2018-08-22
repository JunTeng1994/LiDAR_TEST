#pragma once

//#include <boost/function.hpp>
//#include <boost/bind.hpp>
//#include <boost/thread/thread.hpp>
#include <ctime>

#include "sick_lms400.h"
#include <thread>

using namespace sick_lms400;



class LMS400{
protected: 
public:
	LMS400();
	~LMS400();

	LaserScan getLaserScan();
private:
	
//	boost::shared_ptr<boost::thread> scan_thread_;
	int start();
	int stop();
	void restartMeasurementWithNewValues(float scannig_frequency, float angular_relolution,
		float min_angle, float diff_angle, int intensity,
		bool laser_enabled);
	void getParametersFromConfigFile();
	bool spin();


	SickLMS400 lms_;
	LaserScan scan_;

	std::string hostname_, password_;
	int port_;

	int filter_;
	int mean_filter_params_;
	double range_filter_params_min_, range_filter_params_max_;

	bool intensity_;
	bool laser_enabled_;
	double angular_resolution_, scanning_frequency_;
	double min_angle_, max_angle_;
	int eRIS_;
	
	bool loggedin_;
	int debug_;

	std::mutex m_mutex;
	
};