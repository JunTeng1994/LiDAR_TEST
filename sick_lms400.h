#pragma once
#pragma comment (lib, "ws2_32.lib")

#include <stdlib.h>
#include <stdio.h>
//#include <netdb.h>
#include <string.h>
#include <WinSock2.h>


#include "common_header.h"
#include "scan.h"
#include "angles.h"

#if _MSC_VER
#define snprintf _snprintf
#endif

#define HAVE_GETADDRINFO



#define BUF_SIZE  1024

namespace sick_lms400 {
	typedef struct {
		unsigned char* string;
		int length;
	}MeasurementQueueElement_t;

	typedef struct {
		uint16_t Format;
		uint16_t DistanceScaling;
		int32_t StartingAngle;
		uint16_t AngularStepWidth;
		uint16_t NumberMeasuredValues;
		uint16_t ScanningFrequency;
		uint16_t RemissionScaling;
		uint16_t RemissionStartValue;
		uint16_t RemissionEndValue;
	}MeasurementHeader_t;

	class SickLMS400 {
	public:
		SickLMS400(){}
		SickLMS400 (const char* host, int port, int debug_mode);

		int Connect ();
		int Disconnect ();

		int SetAngularResolution (const char* password, float ang_res, float angle_start, float angle_range);
		int SetScanningFrequency (const char* password, float freq, float angle_start, float angle_range);
		int SetResolutionAndFrequency (float freq, float ang_res, float angle_start, float angle_range);
		
		int StartMeasurement(bool intensity = true);
		LaserScan ReadMeasurement();
		int StopMeasurement();

		int SetUserLevel  (int8_t userlevel, const char* password);
		int GetMACAddress (char** macadress);

		int SetIP         (char* ip);
		int SetGateway    (char* gw);
		int SetNetmask	  (char* mask);
		int SetPort       (uint16_t port);

		int ResetDevice			    ();
		int TerminateConfiguration  ();

		int SendCommand       (const char* cmd);
		int ReadResult        ();
		int ReadAnswer		  ();

		int ReadConfirmationAndAnswer();

		int EnableRIS (int onoff);
		int SetMeanFilterParameters (int num_scans);
		int SetRangeFilterParameters (float range_min, float range_max);
		int EnableFilters (int filter_mask);

		unsigned char* ParseIP(char* ip);

	private:
		int AssembleCommand(unsigned char* command, int len);

		const char *hostname_;
		int sockfd_, portno_, n_;
		struct sockaddr_in serv_addr_;

#ifdef HAVE_GETADDRINFO
		struct addrinfo *addr_ptr_;
#else
		struct hostent *server_;
#endif
		int verbose_;
		int ExtendedRIS_;
		int MeanFilterNumScans_;
		float RangeFilterTopLimit_;
		float RangeFilterBottonLimit_;
		int FilterMask_;

		long int scanning_frequency_, resolution_;

		char buffer_[4096]; // unsigned char 
		unsigned int bufferlength_;

		char command_[BUF_SIZE]; // unsigned char
		int commandlength_;
		std::vector<MeasurementQueueElement_t> *MeasurementQueue_;

		LARGE_INTEGER large_interger; // º∆À„ ±º‰
		double dff;
	};
}
