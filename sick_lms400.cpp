#include "sick_lms400.h"
#include "angles.h"
#include "Point.h"

const int CMD_BUFFER_SIZE = 255;

sick_lms400::SickLMS400::SickLMS400(const char* host, int port, int debug_mode){
	portno_			= port;
	hostname_		= host;
	verbose_		= debug_mode;
	MeasurementQueue_ = new std::vector< MeasurementQueueElement_t >;

	QueryPerformanceFrequency(&large_interger);
	dff = large_interger.QuadPart;
}

int sick_lms400::SickLMS400::Connect(){
//////////////////
//	WSADATA wsaData;
//	WSAStartup(MAKEWORD(2, 2), &wsaData);
//////////////////
	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfd_ < 0) {
		return -1;
	}

	memset(&serv_addr_, 0, sizeof(serv_addr_));
	serv_addr_.sin_port = htons(portno_);
	serv_addr_.sin_family = AF_INET;
	serv_addr_.sin_addr.S_un.S_addr = inet_addr(hostname_);

	if (connect(sockfd_, reinterpret_cast<struct sockaddr*>(&serv_addr_), sizeof(serv_addr_)) < 0){
		return -1;
	}
//#if defined(HAVE_GETADDRINFO)
//	addr_ptr_ = NULL;
//	if (getaddrinfo(hostname_, NULL, NULL, &(addr_ptr_))){
//		std::cout << "getaddringo() failed with error" << std::endl;
//		return -1;
//	}
//	assert(addr_ptr_);
//	assert(addr_ptr_->ai_addr);
//	if ((addr_ptr_->ai_addr->sa_family) != AF_INET){
//		std::cout << "unsupported internet address family" << std::endl;
//		return -1;
//	}
//	serv_addr_.sin_addr.s_addr = (reinterpret_cast<struct sockaddr_in*>(addr_ptr_->ai_addr))->sin_addr.s_addr;
//	freeaddrinfo(addr_ptr);
//	addr_ptr = NULL;
//#else
//		server_ = gethostbyname(hostname_); /////// change
//	if ((server_) == NULL) {
////		return -1;
//	}
//	memcpy(&(serv_addr_.sin_addr.s_addr), server_->h_addr, server_->h_length);
//#endif

//	if (connect(sockfd_, reinterpret_cast<struct sockaddr*>(&serv_addr_), sizeof(serv_addr_)) < 0){
//		return -1;
//	}
	return 0;
}

int sick_lms400::SickLMS400::Disconnect(){
	return (closesocket(sockfd_)); // change close -> closesocket
}

int sick_lms400::SickLMS400::EnableRIS(int onoff){
	char cmd[CMD_BUFFER_SIZE];
	snprintf(cmd, CMD_BUFFER_SIZE, "sWN MDblex %i", onoff); // change snprintf -> _snprintf ///////hxb
	SendCommand(cmd);

	if (ReadAnswer() != 0) {
		return -1;
	}
	ExtendedRIS_ = onoff;
	return 0;
}

int sick_lms400::SickLMS400::SetMeanFilterParameters(int num_scans) {
	char cmd[CMD_BUFFER_SIZE];
	_snprintf_c(cmd, CMD_BUFFER_SIZE, "sWN FLmean 0 %04i", num_scans); // change snprintf -> _snprintf
	SendCommand(cmd);

	if (ReadAnswer() != 0){
		return -1;
	}
	MeanFilterNumScans_ = num_scans;
	return 0;
}

int sick_lms400::SickLMS400::SetRangeFilterParameters(float range_min, float range_max){
	char cmd[CMD_BUFFER_SIZE];
	_snprintf(cmd, CMD_BUFFER_SIZE, "sWN FLrang %+f %+f", (float)range_min, (float)range_max); // change snprintf -> _snprintf
	SendCommand(cmd);
	
	if (ReadAnswer() != 0) {
		return -1;
	}

	RangeFilterBottonLimit_ = range_min;
	RangeFilterTopLimit_ = range_max;
	return 0;

}

int sick_lms400::SickLMS400::EnableFilters(int filter_mask){
	char cmd[CMD_BUFFER_SIZE];
	_snprintf(cmd, CMD_BUFFER_SIZE, "sWN FLsel %+i", filter_mask); // change snprintf -> _snprintf
	SendCommand(cmd);

	if (ReadAnswer() != 0){
		return -1;
	}
	FilterMask_ = filter_mask;
	return 0;
}

unsigned char* sick_lms400::SickLMS400::ParseIP(char *ip){
	char *tmp = (char*)malloc(strlen(ip) + 1);
	unsigned char* _ip = (unsigned char*)malloc(4);

	strcpy(tmp, ip);
	_ip[0] = atoi(strtok(tmp, "."));

	for (int i = 0; i < 4; i++)
	{
		_ip[i] = atoi(strtok(NULL, "."));
	}
	free(tmp);
	return(_ip);
}

int sick_lms400::SickLMS400::SetUserLevel(int8_t userlevel, const char* password){
	char cmd[CMD_BUFFER_SIZE];
	_snprintf(cmd, CMD_BUFFER_SIZE, "sMN SetAccessMode %d %s", userlevel, password); // change snprintf -> _snprintf
	SendCommand(cmd);
	return (ReadConfirmationAndAnswer());
}

int sick_lms400::SickLMS400::GetMACAddress(char** macaddress){
	char *mac = (char*)malloc(20);
	int index = 0;
	char *tmp;

	SendCommand("sRN EImac ");
	if (ReadAnswer() != 0){
		return -1;
	}
	strtok((char*)buffer_, " ");
	strtok(NULL, " ");

	for (int i = 0; i < 6; i++)
	{
		tmp = strtok(NULL, "-");
		strncpy(&mac[index], tmp, 2);
		index += 2;
		mac[index++] =  ':';
	}
	mac[--index] = 0;
	*macaddress = mac;
	return 0;
}

int sick_lms400::SickLMS400::SetIP(char *ip){
	unsigned char* ip_str;
	ip_str = ParseIP(ip);
	char cmd[80];

	_snprintf(cmd, 80, "sWN EIip %X %X %X %X", ip_str[0], ip_str[1], ip_str[2], ip_str[3]); // change snprintf -> _snprintf
	free(ip_str);
	SendCommand(cmd);
	return ReadAnswer();
}

//
int
sick_lms400::SickLMS400::SetGateway(char* gw)
{
	unsigned char* gw_str;
	gw_str = ParseIP(gw);
	char cmd[CMD_BUFFER_SIZE];

	_snprintf(cmd, CMD_BUFFER_SIZE, "sWN EIgate %X %X %X %X", gw_str[0], gw_str[1], gw_str[2], gw_str[3]);
	free(gw_str);
	SendCommand(cmd);

	return (ReadAnswer());
}

int
sick_lms400::SickLMS400::SetNetmask(char* mask)
{
	unsigned char* mask_str;
	mask_str = ParseIP(mask);
	char cmd[CMD_BUFFER_SIZE];

	_snprintf(cmd, CMD_BUFFER_SIZE, "sWN EImask %X %X %X %X", mask_str[0], mask_str[1], mask_str[2], mask_str[3]);
	free(mask_str);
	SendCommand(cmd);

	return (ReadAnswer());
}


int
sick_lms400::SickLMS400::SetPort(uint16_t port)
{
	char cmd[CMD_BUFFER_SIZE];

	_snprintf(cmd, CMD_BUFFER_SIZE, "sWN EIport %04X", port);
	SendCommand(cmd);

	return (ReadAnswer());
}

int
sick_lms400::SickLMS400::ResetDevice()
{
	const char* cmd = "sMN mDCreset ";
	SendCommand(cmd);

	return (ReadAnswer());
}

int
sick_lms400::SickLMS400::TerminateConfiguration()
{
	const char* cmd = "sMN Run";
	SendCommand(cmd);

	return (ReadConfirmationAndAnswer());
}
int
sick_lms400::SickLMS400::SetAngularResolution(const char* password, float ang_res,
float angle_start, float angle_range)
{
	char cmd[CMD_BUFFER_SIZE];
	_snprintf(cmd, CMD_BUFFER_SIZE, "sMN mSCconfigbyang 04 %s %+f 01 %+f %+f",
		password, ang_res, angle_start, angle_range);
	SendCommand(cmd);

	return (ReadConfirmationAndAnswer());
}

int
sick_lms400::SickLMS400::SetScanningFrequency(const char* password, float freq,
float angle_start, float angle_range)
{
	char cmd[CMD_BUFFER_SIZE];
	_snprintf(cmd, CMD_BUFFER_SIZE, "sMN mSCconfigbyfreq 04 %s %+f 01 %+f %+f",
		password, freq, angle_start, angle_range);
	SendCommand(cmd);

	return (ReadConfirmationAndAnswer());
}


int
sick_lms400::SickLMS400::SetResolutionAndFrequency(float freq, float ang_res,
float angle_start, float angle_range)
{
	char cmd[CMD_BUFFER_SIZE];
	_snprintf(cmd, CMD_BUFFER_SIZE, "sMN mSCsetscanconfig %+.2f %+.2f %+.2f %+.2f",
		freq, ang_res, angle_start, angle_range);
	SendCommand(cmd);

	int error = ReadConfirmationAndAnswer();

	// If no error, parse the results
	if (error == 0)
	{
		strtok((char*)buffer_, " "); strtok(NULL, " ");
		int ErrorCode = strtol(strtok(NULL, " "), NULL, 16);
		long int sf = strtol(strtok(NULL, " "), NULL, 16);
		long int re = strtol(strtok(NULL, " "), NULL, 16);

		if ((ErrorCode != 0) && (verbose_ > 0))
			printf(">> Warning: got an error code %d\n", ErrorCode);

		scanning_frequency_ = sf;
		resolution_ = re;

		if (verbose_ > 0)
			printf(">> Measured value quality is: %ld [5-10]\n",
			strtol(strtok(NULL, " "), NULL, 16));
	}

	return (error);
}


int
sick_lms400::SickLMS400::StartMeasurement(bool intensity)
{
	char cmd[CMD_BUFFER_SIZE];
	if (intensity)
		_snprintf(cmd, CMD_BUFFER_SIZE, "sMN mLRreqdata %x", 0x20);
	else
		_snprintf(cmd, CMD_BUFFER_SIZE, "sMN mLRreqdata %x", 0x21);

	SendCommand(cmd);

	return (ReadConfirmationAndAnswer());
}

LaserScan
sick_lms400::SickLMS400::ReadMeasurement()
{
	LaserScan scan;

	char cs_read = 0, cs_calc = 0;
	int length = 0;
	int current = 0;

	memset(buffer_, 0, 256);
	if (!MeasurementQueue_->empty())
	{
		if (verbose_ > 0)
			std::cout << ">>> Reading from queue...\n" << std::endl;
		memcpy(buffer_, (char*)MeasurementQueue_->front().string, MeasurementQueue_->front().length + 1);
		free(MeasurementQueue_->front().string);
		MeasurementQueue_->erase(MeasurementQueue_->begin());
	}
	else
	{
		if (verbose_ == 2)
			std::cout << ">>> Queue empty. Reading from socket...\n" << std::endl;
//		n_ = readsocket(sockfd_, buffer_, 8);
		n_ = recv(sockfd_, buffer_, 8, 0);
		if (n_ < 0)
		{
			if (verbose_ > 0)
				std::cout << ">>> E: error reading from socket!\n" << std::endl;
			return (scan);
		}
		if (buffer_[0] != 0x02 || buffer_[1] != 0x02 || buffer_[2] != 0x02 || buffer_[3] != 0x02)
		{
			if (verbose_ > 0)
				std::cout << ">>>E: error expected 4 bytes STX's!\n" << std::endl;
//			n_ = read(sockfd_, buffer_, 255);
			n_ = recv(sockfd_, buffer_, 255, 0);
			return (scan);
		}

		// find message length
		length = ((buffer_[4] << 24) | (buffer_[5] << 16) | (buffer_[6] << 8) | (buffer_[7]));
		do
		{
//			n_ = read(sockfd_, &buffer_[current], length - current);
			n_ = recv(sockfd_, &buffer_[current], length - current, 0);
			current += n_;
		} while (current < length);

		// read checksum:
//		int ret = read(sockfd_, &cs_read, 1);
		int ret = recv(sockfd_, &cs_read, 1, 0);
		if (ret < 1)
		{
			std::cout << "LMS400 didnt get any data in read " << ret << std::endl;
			return (scan);
		}

		for (int i = 0; i < length; i++)
			cs_calc ^= buffer_[i];

		if (cs_calc != cs_read)
		{
			if (verbose_ > 0)
				std::cout << ">>>E :checksums do not match!\n" << std::endl;
			return (scan);
		}
	}

	// parse measurements header and fill in the configuration parameters
	MeasurementHeader_t meas_header;
	memcpy(&meas_header, (void *)buffer_, sizeof (MeasurementHeader_t));

	float min_angle = meas_header.StartingAngle / 10000.0;
	float resolution = meas_header.AngularStepWidth / 10000.0;
	float max_angle = ((float)meas_header.NumberMeasuredValues) * resolution + min_angle;
	//float scanning_frequency = meas_header.ScanningFrequency;

	if (verbose_ == 2)
		std::cout << ">>> Reading " << meas_header.NumberMeasuredValues << " values from " << meas_header.StartingAngle / 10000.0 <<
		" to " << ((float)meas_header.NumberMeasuredValues) * resolution + min_angle << std::endl;
//		ROS_DEBUG(">>> Reading %d values from %f to %f", meas_header.NumberMeasuredValues, meas_header.StartingAngle / 10000.0,
//		((float)meas_header.NumberMeasuredValues) * resolution + min_angle);

	uint16_t distance;
	uint8_t remission;
	int index = sizeof (MeasurementHeader_t);

	// Fill in the appropriate values
	scan.angle_min = angles::from_degrees(min_angle);
	scan.angle_max = angles::from_degrees(max_angle);
	scan.angle_increment = angles::from_degrees(resolution);
	scan.range_min = 0.7;
	scan.range_max = 3.6;
	scan.ranges.resize(meas_header.NumberMeasuredValues);
	scan.intensities.resize(meas_header.NumberMeasuredValues);

	memcpy(&scan.scan_time, &buffer_[sizeof(MeasurementHeader_t)+meas_header.NumberMeasuredValues * 3 + 14], 2);

	// Parse the read buffer and copy values into our distance/intensity buffer
	for (int i = 0; i < meas_header.NumberMeasuredValues; i++)
	{
		if (meas_header.Format == 0x20 || meas_header.Format == 0x21)
		{
			memcpy(&distance, (void *)&buffer_[index], sizeof (uint16_t));
			index += sizeof (uint16_t);
		}
		if (meas_header.Format == 0x20 || meas_header.Format == 0x22)
		{
			memcpy(&remission, (void *)&buffer_[index], sizeof (uint8_t));
			index += sizeof (uint8_t);
		}
//		scan.ranges[i] = distance * meas_header.DistanceScaling / 1000.0;
		double d = distance * meas_header.DistanceScaling / 1000.0;
//		scan.push_back(Point::Polar(d, scan.angle_min + scan.angle_increment * i));
		scan.ranges[i] = Point::Polar(d, scan.angle_min + scan.angle_increment * i);
	//	std::cout << scan.size() << std::endl;
		scan.intensities[i] = remission * meas_header.RemissionScaling;

		if (verbose_ == 2)
			std::cout << ">>> [" << i << "] dist : " << distance * meas_header.DistanceScaling << "\t remission: " << remission * meas_header.RemissionScaling << std::endl;

//			ROS_DEBUG(" >>> [%i] dist: %i\t remission: %i", i, distance * meas_header.DistanceScaling, remission * meas_header.RemissionScaling);
	}

	scan.header.frame_id = "lms400_tilt_laser";

	QueryPerformanceCounter(&large_interger);
	scan.header.stamp = large_interger.QuadPart * 1000.0 / dff;
/////////////////////////////测试时间/////////////////////////////////////////
//	static std::ofstream *outfile = new std::ofstream("time.txt", std::ios::end);
//	*outfile << "scan.header.stamp : " << scan.header.stamp << std::endl;
/////////////////////////////测试时间/////////////////////////////////////////

//	scan.header.stamp = ros::Time::now();
	return (scan);
}


int
sick_lms400::SickLMS400::StopMeasurement()
{
	char cmd[CMD_BUFFER_SIZE];
	_snprintf(cmd, CMD_BUFFER_SIZE, "sMN mLRstopdata");
	SendCommand(cmd);

	return (ReadConfirmationAndAnswer());
}


int
sick_lms400::SickLMS400::SendCommand(const char* cmd)
{
	if (verbose_ > 0)
		std::cout << ">> Sent: \"" << cmd << "\"" << std::endl;
//		ROS_DEBUG(">> Sent: \"%s\"\n", cmd);
	AssembleCommand((unsigned char *)cmd, strlen(cmd));

//	n_ = write(sockfd_, command_, commandlength_);
	n_ = send(sockfd_, command_, commandlength_, 0);
	if (n_ < 0)
		return (-1);

	return (0);
}


int
sick_lms400::SickLMS400::ReadResult()
{
	memset(buffer_, 0, 256);
	n_ = recv(sockfd_, buffer_, 8, 0);
//	n_ = read(sockfd_, buffer_, 8);
	if (n_ < 0)
		return (-1);

	if (buffer_[0] != 0x02 || buffer_[1] != 0x02 || buffer_[2] != 0x02 || buffer_[3] != 0x02)
	{
		if (verbose_ > 0)
			std::cout << ">E: expected 4 bytes STX's!" << std::endl;
//			ROS_WARN("> E: expected 4 bytes STX's!");
		n_ = recv(sockfd_, buffer_, 255, 0);
		//n_ = read(sockfd_, buffer_, 255);
		return (-1);
	}

	// Find message length
	int length = ((buffer_[4] << 24) | (buffer_[5] << 16) | (buffer_[6] << 8) | (buffer_[7]));
	int current = 0;
	do
	{
		n_ = recv(sockfd_, &buffer_[current], length - current, 0);
	//	n_ = read(sockfd_, &buffer_[current], length - current);
		current += n_;
	} while (current < length);

	bufferlength_ = length;
	if ((verbose_ > 0) && (buffer_[0] != 0x20))
		std::cout << ">> Received: \"" << buffer_ << "\"" << std::endl;
//		ROS_DEBUG(">> Received: \"%s\"\n", buffer_);

	// Check for error
	if (strncmp((const char*)buffer_, "sFA", 3) == 0)
	{
		strtok((char*)buffer_, " ");
		std::cout << ">>E: Got an error message with code 0x" << strtok(NULL, " ") << std::endl;
//		ROS_DEBUG(">> E: Got an error message with code 0x%s\n", strtok(NULL, " "));
	}

	// Read checksum:
	char cs_read = 0;
	int ret = recv(sockfd_, &cs_read, 1, 0);
//	int ret = read(sockfd_, &cs_read, 1);
	if (ret < 1)
	{
		std::cout << "LMS400 didnt get any data in read " << ret << std::endl;
//		ROS_ERROR("LMS400 didnt get any data in read %d", ret);
		return (-1);
	}

	if (buffer_[0] == 's')
		return (0);
	else if (buffer_[0] == 0x20)
		return (ReadResult());
	else if (bufferlength_ > sizeof (MeasurementHeader_t))
	{
		if (verbose_ > 0)
			std::cout << ">>>> ReadResult: probably found a data packet!\n>>>>    " << buffer_ << std::endl;
//			ROS_DEBUG(">>>> ReadResult: probably found a data packet!\n>>>>             %s\n", buffer_);
		// Don't throw away our precious measurement, queue it for later use :)
		unsigned char* tmp = (unsigned char*)malloc(bufferlength_ + 1);
		memcpy(tmp, buffer_, bufferlength_ + 1);
		MeasurementQueueElement_t q;
		q.string = tmp;
		q.length = bufferlength_;
		MeasurementQueue_->push_back(q);
		// and then, try to read what we actually wanted to read...
		return (ReadResult());
	}

	return (0);
}

int
sick_lms400::SickLMS400::ReadAnswer()
{
	return (ReadResult());
}

int
sick_lms400::SickLMS400::ReadConfirmationAndAnswer()
{
	ReadResult();
	if (buffer_[0] == 's' && buffer_[1] == 'F' && buffer_[2] == 'A')
		return (-1);
	else
		return (ReadResult());
}

int
sick_lms400::SickLMS400::AssembleCommand(unsigned char* cmd, int len)
{
	unsigned char checksum = 0;
	int index = 0;

	command_[0] = 0x02;  // Messages start with 4 STX's
	command_[1] = 0x02;
	command_[2] = 0x02;
	command_[3] = 0x02;
	command_[4] = (len >> 24) & 0xff; // then message length
	command_[5] = (len >> 16) & 0xff;
	command_[6] = (len >> 8) & 0xff;
	command_[7] = (len)& 0xff;

	for (index = 0; index < len; index++)
	{
		command_[index + 8] = cmd[index];
		checksum ^= cmd[index];
	}
	command_[8 + len] = checksum;
	command_[9 + len] = 0x00;

	commandlength_ = 9 + len;
	return (0);
}
