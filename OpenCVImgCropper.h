#pragma once

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>

using namespace std;

struct Coordinate {
	int x;
	int y;
};

struct Tuple {
	cv::Mat face;
	Coordinate s_point;
};

class OpenCVImgCropper {
public:
	OpenCVImgCropper(int w, int h);
	OpenCVImgCropper();
	void init();

	cv::Mat getOriginalImg(string filePathOfImg);

	cv::Mat convertToMat(unsigned int height, unsigned int width, unsigned char* stream);
	Tuple getMatOfFace(string filePathOfImg);
	Tuple getMatOfFace(unsigned int height, unsigned int width, unsigned char* stream);
	Tuple getMatOfFace(cv::Mat& frame);

private:
	cv::dnn::Net net;
	const string modelDesc = "../Model/OpenCV/opencv_face_detector.pbtxt";
	const string modelBinary = "../Model/OpenCV/opencv_face_detector_uint8.pb";
	size_t inWidth = 300;
	size_t inHeight = 300;
	const double inScaleFactor = 1.0;
	const float confidenceThreshold = 0.7;
};