#include "LMS400.h"

//#define LMS_LINES_LEN  700
//#define LMS_TRACJECTORY_LEN 10000
//
//struct LMS_Data {
//	double lines[LMS_LINES_LEN];
//	double time;
//};
//
//class LMS_Trajectory
//{
//public:
//	LMS_Trajectory();
//	~LMS_Trajectory();
//
//	int Append(LMS_Data dat, int rm_mode = 1);
//	int Read(LMS_Data &dat, int rm_mode = 1);
//	int Size();
//	int Get(std::vector<LMS_Data> &vec);
//	int Clear();
//
//private:
//	int head, tail;
//	LMS_Data *pData;
//};
//
//LMS_Trajectory::LMS_Trajectory()
//{
//	head = tail = 0;
//	pData = new LMS_Data[LMS_TRACJECTORY_LEN];
//}
//
//LMS_Trajectory::~LMS_Trajectory()
//{
//	delete[] pData;
//}
//
//int LMS_Trajectory::Read(LMS_Data &dat, int rm_mode)
//{
//	int t_head = (head + 1) % LMS_TRACJECTORY_LEN;
//	if (t_head == tail) return 0;
//	memcpy(&dat, &pData[head], sizeof(LMS_Data));
//	if (rm_mode) head = t_head;
//	return 1;
//}
//
//int LMS_Trajectory::Append(LMS_Data dat, int rm_mode)
//{
//	int t_tail = (tail + 1) % LMS_TRACJECTORY_LEN;
//	if (t_tail == head) {
//		if (rm_mode == 0) return 0;
//		head = (head + 1) % LMS_LINES_LEN;
//	}
//	memcpy(&pData[tail], &dat, sizeof(LMS_Data));
//	tail = t_tail;
//	return 1;
//}
//
//int LMS_Trajectory::Get(std::vector<LMS_Data> &vec)
//{
//	int n = Size();
//	vec.clear();
//	if (n <= 0) return 0;
//	vec.resize(n);
//	if (tail > head) memcpy(&vec[0], &pData[head], sizeof(LMS_Data) * n);
//	memcpy(&vec[0], &pData[head], (LMS_TRACJECTORY_LEN - head) * sizeof(LMS_Data));
//	memcpy(&vec[LMS_TRACJECTORY_LEN - head], &pData[0], tail* sizeof(LMS_Data));
//	Clear();
//	return n;
//}
//
//int LMS_Trajectory::Clear()
//{
//	tail = head;
//	return 1;
//}
//
//int LMS_Trajectory::Size()
//{
//	int len = tail - head;
//	if (len < 0) len += LMS_TRACJECTORY_LEN;
//	return len;
//}
//

LMS400::LMS400() :
debug_(0),
hostname_("192.168.0.1"),
password_("81BE23AA"),
port_(2111),
filter_(3),
eRIS_(1),
mean_filter_params_(2),
range_filter_params_min_(700.0),
range_filter_params_max_(3000.0),
angular_resolution_(0.3636),
scanning_frequency_(500),
laser_enabled_(true),
intensity_(true),
min_angle_(55.0),
max_angle_(70.0),
loggedin_(false)
{
	start();
	static std::thread t(&LMS400::spin, this);
}

LMS400::~LMS400(){
	stop();
}

int LMS400::start(){
		lms_ = SickLMS400(hostname_.c_str(), port_, debug_);
		if (lms_.Connect() != 0){
			std::cout << " [SickLMS400] Connecting to SICK LMS400 on [" << hostname_.c_str() <<
				":" << port_ << "]...[failed!]" << std::endl;
			return -1;
		}
			std::cout << " [SickLMS400] Connecting to SICK LMS400 on [" << hostname_.c_str() <<
				":" << port_ << "]...[done!]" << std::endl;
			lms_.StopMeasurement();
			if (strncmp(password_.c_str(), "NULL", 4) != 0){
//				if (0){
				if (lms_.SetUserLevel(4, password_.c_str()) != 0){
					std::cout << "> [SickLMS400] Unable to change userlevel to 'Service' using " << password_.c_str() << std::endl;
				}
				else {
					loggedin_ = true;
					if ((mean_filter_params_ >= 2) && (mean_filter_params_ <= 200)){
						lms_.SetMeanFilterParameters(mean_filter_params_);
					}
					if ((range_filter_params_min_ >= 700) && (range_filter_params_max_ > range_filter_params_min_)){
						lms_.SetRangeFilterParameters((float)range_filter_params_min_, (float)range_filter_params_max_);
					}
					lms_.EnableFilters(filter_);
					std::cout << "> [SickLMS400] Enabling selected filters (" << mean_filter_params_ << ", " << (float)range_filter_params_min_ <<
						", " << (float)range_filter_params_max_ << ", " << filter_ <<  ")...[done]" << std::endl;
				}
			}
			else {
				std::cout << "> [SickLMS400] Userlevel 3 password not given. Filter(s) disabled!" << std::endl;
			}
			if (eRIS_){
				lms_.EnableRIS(1);
				std::cout << "> [SickLMS400] Enabling extended RIS detectivity... [done]" << std::endl;
			}

			if (lms_.SetResolutionAndFrequency(scanning_frequency_, angular_resolution_, min_angle_, max_angle_) != 0) {
				std::cout << "> [SickLMS400] Couldn't set values for resolution, frequency, and min/max angle. Using previously set values." << std::endl;
			}
			else {
				std::cout << "> [SickLMS400] Enabling user values for resolution (" << angular_resolution_ << "), frequency (" << scanning_frequency_ << ") and min/max angle (" << min_angle_ << "/" << max_angle_ << ")...[done]" << std::endl;
			}
			return 0;
}

int LMS400::stop(){
		lms_.StopMeasurement();
		lms_.TerminateConfiguration();
		lms_.Disconnect();
		std::cout << "> [SickLMS400] SICK LMS400 driver shutting down... [done]" << std::endl;
		return 0;
}

void LMS400::restartMeasurementWithNewValues(float scanning_frequency, float angular_resolution,
	float min_angle, float diff_angle, int intensity, bool laser_enabled){
		lms_.StopMeasurement();
		if (lms_.SetUserLevel(4, password_.c_str()) != 0){
			std::cout << "> Unable to change userlevel to 'Service' using " << password_.c_str() << std::endl;
			if (laser_enabled){
				lms_.StartMeasurement(intensity);
			}
		}
		else {
			if (lms_.SetResolutionAndFrequency(scanning_frequency, angular_resolution, min_angle, diff_angle) == 0){
				if (laser_enabled){
					lms_.StartMeasurement(intensity);
				}
			}
		}
}

void LMS400::getParametersFromConfigFile(){
}


bool LMS400::spin(){
//	LMS_Trajectory traj;
	lms_.StartMeasurement(intensity_);

	bool first_scan_flag = true;
	int count = 0;
	long double sum = 0.0;
	long double lastTime = 0.0;
	long double currTime = 0.0;
	while (true)
	{
		std::lock_guard<std::mutex> lg(m_mutex);
		scan_ = lms_.ReadMeasurement();

	}
	return true;
}

LaserScan LMS400::getLaserScan(){
	std::lock_guard<std::mutex> lg(m_mutex);
	return scan_;
}