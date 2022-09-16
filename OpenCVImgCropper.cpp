#include "OpenCVImgCropper.h"
#include "common.h"

const cv::Scalar meanVal(104.0, 177.0, 123.0);

void OpenCVImgCropper::init() {
	net = cv::dnn::readNetFromTensorflow(modelBinary, modelDesc);
	int cudaCount = cv::cuda::getCudaEnabledDeviceCount();
	if (cudaCount > 0) {
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
	}
	else {
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	}

	if (net.empty()) {
		cout << "could not load net..." << endl;
		throw "could not load net...";
	}
}

OpenCVImgCropper::OpenCVImgCropper(int w, int h) {
	inWidth = w; 
	inHeight = h;
	init();
}

OpenCVImgCropper::OpenCVImgCropper() {
	init();
}

cv::Mat OpenCVImgCropper::convertToMat(unsigned int height, unsigned int width, unsigned char* stream) {
	cv::Mat frame = cv::Mat(height, width, CV_8UC4, stream);
	cvtColor(frame, frame, cv::COLOR_RGBA2BGR);
	return frame;
}

cv::Mat OpenCVImgCropper::getOriginalImg(string filePathOfImg) {
	return imread(filePathOfImg, cv::IMREAD_COLOR);
}

Tuple OpenCVImgCropper::getMatOfFace(string filePathOfImg) {
	return getMatOfFace(imread(filePathOfImg, cv::IMREAD_COLOR));
}

Tuple OpenCVImgCropper::getMatOfFace(unsigned int height, unsigned int width, unsigned char* stream) {
	return getMatOfFace(convertToMat(height, width, stream));
}

Tuple OpenCVImgCropper::getMatOfFace(cv::Mat& frame) {
	int count = 0;
	Coordinate p = {};

	//int64 start = getTickCount(); // timer calculating how long it takes

#if (OPENCV_DEBUG_MODE)
		imshow("input", frame);
#endif

	if (frame.channels() == 4) // RGBA
		cvtColor(frame, frame, cv::COLOR_BGRA2BGR);

	cout << "type changed!!!!!!!!!" << endl;


	cv::Mat inputBlob = cv::dnn::blobFromImage(frame, inScaleFactor, cv::Size(inWidth, inHeight), meanVal, false, false);
	net.setInput(inputBlob, "data");

	cv::Mat detection = net.forward("detection_out");
	std::vector<double> layersTimings;

	//double freq = getTickFrequency() / 1000;
	//double time = net.getPerfProfile(layersTimings) / freq;

	cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
	//cout << "------------" << detectionMat.rows << endl;
	ostringstream ss;
	cv::Rect returnedRect;
	for (int i = 0; i < detectionMat.rows; ++i) {
		float  confidence = detectionMat.at<float>(i, 2);
		if (confidence > confidenceThreshold) {
			count++;
			int xLeftTop = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols) - 20; // - 20
			int yLeftTop = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows) - 20; // - 30 
			int xRightBottom = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols) + 20; // + 20
			int yRightBottom = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows) + 20;
			cout << "----- OpenCV Face Detector Start -----" << endl;
			cout << "OpenCV >> xLeftTop: " << xLeftTop << endl;
			cout << "OpenCV >> yLeftTop: " << yLeftTop << endl;
			cout << "OpenCV >> width: " << xRightBottom - xLeftTop << endl;
			cout << "OpenCV >> height: " << yRightBottom - yLeftTop << endl;
			cout << "----- OpenCV Face Detector End -----" << endl;

			p = { xLeftTop, yLeftTop };

			cv::Rect object((int)xLeftTop, (int)yLeftTop,
				(int)(xRightBottom - xLeftTop),
				(int)(yRightBottom - yLeftTop));
			rectangle(frame, object, cv::Scalar(0, 255, 0));
			returnedRect = object;
#if (OPENCV_DEBUG_MODE)
			cv::Mat croppedRef(frame, object);
			cv::Mat cropped;
			croppedRef.copyTo(cropped);
			imshow("Cropped Image", croppedRef);
#endif
		}
	}
	//ss.str("");
	//ss << "inference time: " << time << " ms";
	//putText(frame, ss.str(), cv::Point(20, 20), 0, 0.75, cv::Scalar(0, 0, 255), 2, 8);

#if (OPENCV_DEBUG_MODE)
	imshow("dnn_face_detect", frame);
	cout << "X: " << returnedRect.x << ", Y: " << returnedRect.y << endl;
	cout << "Width: " << returnedRect.width << ", Height: " << returnedRect.height << endl;
#endif

	return { cv::Mat(frame, returnedRect), p };
}