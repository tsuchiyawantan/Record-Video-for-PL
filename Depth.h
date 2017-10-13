#pragma once
#include <iostream>
#include <sstream>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <atlbase.h>
#include "NtKinect.h"


#ifdef USE_AUDIO
#include "WaveFile.h"
#endif /* USE_AUDIO */

using namespace std;

#define HIGHT 200
#define WIDTH 200

class Depth : public NtKinect

{
	// ******** depth *******
private:
public:
	int depthMax;
	int depthMin;
	cv::Mat bodyDepthImage;
	cv::Mat bodyOneDepthImage;
	cv::Mat normalizeDepthImage;
	cv::Mat normalizeOneDepthImage;
	cv::Mat contourImage;

	void findDepthMaxMin(int y, int x, int a){
		if (depthMax < a) depthMax = a;
		if (depthMin > a) depthMin = a;
	}

	void setBodyDepth(){
		updateDepthFrame();
		updateBodyIndexFrame();
		bodyDepthImage = cv::Mat(depthHeight, depthWidth, CV_16UC1);
		depthMax = -1;
		depthMin = INFINITE;
		for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
			int y = i / bodyIndexWidth;
			int x = i % bodyIndexWidth;
			if (bodyIndexBuffer[i] == 255) {
				bodyDepthImage.at<UINT16>(y, x) = 65535;
			}
			else {
				bodyDepthImage.at<UINT16>(y, x) = depthBuffer[i];
				findDepthMaxMin(y, x, depthBuffer[i]);
			}
		}
	}

	void setOneBodyDepth(vector<int> people_index){
		updateDepthFrame();
		bodyOneDepthImage = cv::Mat(depthHeight, depthWidth, CV_16UC1);
		depthMax = -1;
		depthMin = INFINITE;
		vector<int> record;
		for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
			int y = i / bodyIndexWidth;
			int x = i % bodyIndexWidth;
			if (bodyIndexBuffer[i] != 255 && people_index.at(bodyIndexBuffer[i]) == 1){
					record.push_back(bodyIndexBuffer[i]);
					bodyOneDepthImage.at<UINT16>(y, x) = depthBuffer[i];
					findDepthMaxMin(y, x, depthBuffer[i]);
			}
			else {
				bodyOneDepthImage.at<UINT16>(y, x) = 65535;
			}
		}
	}

	void setNormalizeDepth(cv::Mat &srcImg){
		normalizeDepthImage = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC1);
		for (int i = 0; i < srcImg.rows*srcImg.cols; i++) {
			int y = i / srcImg.cols;
			int x = i % srcImg.cols;
			if (bodyIndexBuffer[i] == 255) normalizeDepthImage.at<UCHAR>(y, x) = 255;
			else {
				normalizeDepthImage.at<UCHAR>(y, x) = (199 * (srcImg.at<UINT16>(y, x)) / (depthMax - depthMin));
			}
		}
	}
	void setOneNormalizeDepth(cv::Mat &srcImg, vector<int> people_index){
		normalizeOneDepthImage = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC1);
		for (int i = 0; i < srcImg.rows*srcImg.cols; i++) {
			int y = i / srcImg.cols;
			int x = i % srcImg.cols;
			if (bodyIndexBuffer[i] != 255 && people_index.at(bodyIndexBuffer[i]) == 1){
					normalizeOneDepthImage.at<UCHAR>(y, x) = (199 * (srcImg.at<UINT16>(y, x)) / (depthMax - depthMin));
			}
			else normalizeOneDepthImage.at<UCHAR>(y, x) = 255;
		}
	}

	void setContour(cv::Mat &srcImg){
		cv::Mat image2 = srcImg.clone();
		cv::dilate(image2, image2, cv::Mat(), cv::Point(-1, -1), 1);
		cv::erode(image2, image2, cv::Mat(), cv::Point(-1, -1), 1);
		cv::GaussianBlur(image2, image2, cv::Size(5, 5), 0, 0);
		contourImage = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC1);
		cv::Canny(image2, contourImage, 10, 30);
	}

	//Find people in the frame
	bool findPeople(vector<int> &people_index){
		bool flag = false;
		for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
			if (bodyIndexBuffer[i] != 255) {
				people_index.at(bodyIndexBuffer[i]) = 1;
				flag = true;
			}
		}
		return flag;
	}

	bool findPeopleIndex(vector<int> &people_index){
		bool flag = false;
		vector<int> dummy_people_index(people_index.size(), -1);
		for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
			int y = i / bodyIndexWidth;
			int x = i % bodyIndexWidth;
			if (bodyIndexBuffer[i] != 255){
				if (people_index.at(bodyIndexBuffer[i]) == 1) {
					flag = true;
					dummy_people_index.at(bodyIndexBuffer[i]) = 1;
				}
			}
		}
		int i = 0;
		for (auto itr = people_index.begin(), itr_d = dummy_people_index.begin();
			itr != people_index.end(), itr_d != dummy_people_index.end(); ++itr, ++itr_d){
			if (*itr != *itr_d) people_index.at(i) = -1;
			i++;
		}
		dummy_people_index.clear();
		dummy_people_index.shrink_to_fit();

		return flag;
	}
};