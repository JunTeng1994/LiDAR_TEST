#pragma once
#include <stdint.h>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <assert.h>
#include <mutex>
#define NOMINMAX // change
#include <Windows.h>

class ParameterReader {
public:
	ParameterReader(std::string filename = "parameters.txt"){
		std::ifstream fin(filename.c_str());
		if (!fin)
		{
			std::cout << filename << " file does not exist." << std::endl;
			return;
		}
		while (!fin.eof()){
			std::string str;
			getline(fin, str);
			if (str[0] == '#'){
				continue;
			}
			int pos = str.find("=");
			if (pos == -1){
				continue;
			}
			std::string key = str.substr(0, pos);
			std::string value = str.substr(pos + 1, str.length());
			data[key] = value;
			std::cout << key << " : " << data[key] << std::endl;
			if (!fin.good()){
				break;
			}
		}
	}
	std::string getData(std::string key){
		std::map<std::string, std::string>::iterator iter = data.find(key);
		if (iter == data.end()){
			std::cout << "Parameter name " << key << " not found!" << std::endl;
			return std::string("NOT_FOUND");
		}
		return iter->second;
	}
public:
	std::map<std::string, std::string> data;
};

