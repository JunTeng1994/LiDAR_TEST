///*
#define NOMINMAX
#include<iostream>
#include<stdio.h>
#include<fstream>
#include "LMS400.h"
#include "my_pcl.h"
#include "laserScan2PCL.h"

using namespace std;

#define test


int main(void){
#ifdef test
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	LaserScan2PCL* laserScan2PCL = new LaserScan2PCL();

	WSACleanup();
	getchar();
#else
	int i = 4;
	printf("%04d", i);
	getchar();
#endif
	return 0;
}

