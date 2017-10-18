#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>

using namespace std;

class People{
private:
	vector<cv::Mat> pics;

public:

	People(){}
	~People(){}

	void createVideo(int num){
		tm newTime;
		__time64_t longTime;

		_time64(&longTime);
		_localtime64_s(&newTime, &longTime);

		// current date/time based on current system
		int month = newTime.tm_mon + 1;
		int day = newTime.tm_mday;
		int hour = newTime.tm_hour;
		int min = newTime.tm_min;
		int sec = newTime.tm_sec;
		string date = to_string(month) + to_string(day) + to_string(hour) + to_string(min) + to_string(sec);

		int width = (*pics.begin()).cols;
		int hight = (*pics.begin()).rows;
		cv::VideoWriter writer("ppl_"+to_string(num)+".avi", cv::VideoWriter::fourcc('I', '4', '2', '0'), 10, cv::Size(width, hight), true);
		if (!writer.isOpened()){
			cout << "Error!! Unable to open video file for output." << endl;
			exit(-1);
		}
		for (auto itr = pics.begin(); itr != pics.end(); ++itr){
			cv::cvtColor(*itr, *itr, cv::COLOR_GRAY2BGR);
			writer << *itr;
		}

		
		pics.clear();
		pics.shrink_to_fit();
	}

	void showPics(){
		int i = 0;
		for (auto itr = pics.begin(); itr != pics.end(); ++itr){
			cv::imwrite("pictest/pic" + to_string(i++) + ".jpg", *itr);
			cv::imshow("picspicspics", *itr);
		}
	}

	void setPicture(cv::Mat src_image){
		pics.push_back(src_image);
	}

	bool hasPics(){
		if (pics.empty()) return false;
		return true;
	}
};