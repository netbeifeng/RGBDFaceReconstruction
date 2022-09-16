#pragma once

#include <vector>
#include <iomanip>
#include <iostream>
#include <cstring>
#include <fstream>
#include "OpenCVImgCropper.h"
#include "LandmarkDetector.h"
#include "Eigen.h"
#include "FreeImageHelper.h"


typedef unsigned char BYTE;

enum DATASET_TYPE { FACE_GRABBER, KINECT_TUM_VISION_GROUP, KINECT_V2, REALSENSE };

// reads sensor files according to https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
class VirtualSensor
{
public:

	VirtualSensor() : m_currentIdx(-1), m_increment(10)
	{
		m_isCamMoving = true;

		m_dataType = KINECT_TUM_VISION_GROUP;

		// image resolutions
		m_colorImageWidth = 640;
		m_colorImageHeight = 480;
		m_depthImageWidth = 640;
		m_depthImageHeight = 480;
		m_depthScaleFactor = 1000;
	}

	VirtualSensor(unsigned int colorImageWidth, unsigned int colorImageHeight,
		unsigned int depthImageWidth, unsigned int depthImageHeight, int increment, bool isCamMoving, DATASET_TYPE dataType, int numFrame) : m_currentIdx(-1),
		m_colorImageWidth(colorImageWidth), m_colorImageHeight(colorImageHeight),
		m_depthImageWidth(depthImageWidth), m_depthImageHeight(depthImageHeight),
		m_increment(increment), m_isCamMoving(isCamMoving), m_dataType(dataType),
		m_numFrame(numFrame),
		m_depthScaleFactor(1000)
	{

	}

	~VirtualSensor()
	{
		SAFE_DELETE_ARRAY(m_depthFrame);
		SAFE_DELETE_ARRAY(m_colorFrame);
	}

	bool Init(const std::string& datasetDir)
	{
		m_baseDir = datasetDir;
		switch (m_dataType)
		{
		case KINECT_TUM_VISION_GROUP:
		{
			if (!ReadFileList(datasetDir + "depth.txt", m_filenameDepthImages, m_depthImagesTimeStamps)) return false;
			if (!ReadFileList(datasetDir + "rgb.txt", m_filenameColorImages, m_colorImagesTimeStamps)) return false;
			if (!ReadTrajectoryFile(datasetDir + "groundtruth.txt", m_trajectory, m_trajectoryTimeStamps)) return false;
			// depth images are scaled by 5000 for Kinect data
			// (see https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)
			m_depthScaleFactor = 5000; // mm to meters
			break;
		}

		case FACE_GRABBER:
		{
			readDepthAndColourImageNames("KinectDepthRaw/Kinect_Depth_Raw_",
				 "KinectColorRaw/Kinect_Color_Raw_",
				".png",
				".jpeg");

			Eigen::Matrix4f transf;
			transf = transf.setIdentity().inverse().eval();
			m_trajectory.push_back(transf);
			m_depthScaleFactor = 1000; // mm to meters
			break;
		}

		case KINECT_V2:
		{
			readDepthAndColourImageNames("depth/",
				"rgb/",
				".png",
				".png");

			Eigen::Matrix4f transf;
			transf = transf.setIdentity().inverse().eval();
			m_trajectory.push_back(transf);
			m_depthScaleFactor = 1000; // mm to meters
			break;
		}

		case REALSENSE:
		{
			readDepthAndColourImageNames("depth/",
				"rgb/",
				".png",
				".png");

			Eigen::Matrix4f transf;
			transf = transf.setIdentity().inverse().eval();
			m_trajectory.push_back(transf);
			// RealSenseL515 z unit is 0.00025m (1/4000)
			// (https://github.com/IntelRealSense/librealsense/issues/8058)
			m_depthScaleFactor = 4000; // to meters
			break;
		}

		default:
			std::cout << "Unsupported data type!" << std::endl;
			break;
		}

		if (m_filenameDepthImages.size() != m_filenameColorImages.size()) return false;

		// intrinsics
		m_colorIntrinsics <<	525.0f, 0.0f, 319.5f,
								0.0f, 525.0f, 239.5f,
								0.0f, 0.0f, 1.0f;

		m_depthIntrinsics = m_colorIntrinsics;

		m_colorExtrinsics.setIdentity();
		m_depthExtrinsics.setIdentity();

		m_depthFrame = new float[m_depthImageWidth*m_depthImageHeight];
		for (unsigned int i = 0; i < m_depthImageWidth*m_depthImageHeight; ++i) m_depthFrame[i] = 0.5f;

		m_colorFrame = new BYTE[4* m_colorImageWidth*m_colorImageHeight];
		for (unsigned int i = 0; i < 4*m_colorImageWidth*m_colorImageHeight; ++i) m_colorFrame[i] = 255;


		m_currentIdx = -1;
		return true;
	}

	bool ProcessNextFrame()
	{
		if (m_currentIdx == -1)	m_currentIdx = 0;
		else m_currentIdx += m_increment;

		if ((unsigned int)m_currentIdx >= (unsigned int)m_filenameColorImages.size()) return false;

		std::cout << "ProcessNextFrame [" << m_currentIdx << " | " << m_filenameColorImages.size() << "]" << std::endl;

		FreeImageB rgbImage;
		FreeImageU16F dImage;

		std::string curr_rgb_path = m_baseDir + m_filenameColorImages[m_currentIdx];
		std::string curr_depth_path = m_baseDir + m_filenameDepthImages[m_currentIdx];
		if (!exists(curr_rgb_path))
		{
			std::cout << "File (" << curr_rgb_path << ") does not exists!" << std::endl;
			return false;
		}
		if (!exists(curr_depth_path))
		{
			std::cout << "File (" << curr_depth_path << ") does not exists!" << std::endl;
			return false;
		}
		rgbImage.LoadImageFromFile(curr_rgb_path);

		std::cout << curr_rgb_path << " loaded" << std::endl;
		memcpy(m_colorFrame, rgbImage.data, 4 * m_colorImageWidth * m_colorImageHeight);

		dImage.LoadImageFromFile(curr_depth_path);
		std::cout << curr_depth_path << " loaded" << std::endl;
		for (unsigned int i = 0; i < m_depthImageWidth * m_depthImageHeight; ++i)
		{
			if (dImage.data[i] == 0)
				m_depthFrame[i] = MINF;
			else
				m_depthFrame[i] = dImage.data[i] * 1.0f / m_depthScaleFactor;
		}

		// find transformation (simple nearest neighbor, linear search)
		if (m_isCamMoving)
		{
			double timestamp = m_depthImagesTimeStamps[m_currentIdx];
			double min = std::numeric_limits<double>::max();
			int idx = 0;
			for (unsigned int i = 0; i < m_trajectory.size(); ++i)
			{
				double d = fabs(m_trajectoryTimeStamps[i] - timestamp);
				if (min > d)
				{
					min = d;
					idx = i;
				}
			}
			m_currentTrajectory = m_trajectory[idx];
		}

		return true;
	}

	unsigned int GetCurrentFrameCnt()
	{
		return (unsigned int)m_currentIdx;
	}

	// get current color data
	BYTE* GetColorRGBX()
	{
		return m_colorFrame;
	}
	// get current depth data
	float* GetDepth()
	{
		return m_depthFrame;
	}

	// color camera info
	Eigen::Matrix3f GetColorIntrinsics()
	{
		return m_colorIntrinsics;
	}

	Eigen::Matrix4f GetColorExtrinsics()
	{
		return m_colorExtrinsics;
	}

	unsigned int GetColorImageWidth()
	{
		return m_colorImageWidth;
	}

	unsigned int GetColorImageHeight()
	{
		return m_colorImageHeight;
	}

	std::string GetColorImagePath()
	{
		return  m_baseDir + m_filenameColorImages[m_currentIdx];
	}

	// depth (ir) camera info
	Eigen::Matrix3f GetDepthIntrinsics()
	{
		return m_depthIntrinsics;
	}

	Eigen::Matrix4f GetDepthExtrinsics()
	{
		return m_depthExtrinsics;
	}

	unsigned int GetDepthImageWidth()
	{
		return m_depthImageWidth;
	}

	unsigned int GetDepthImageHeight()
	{
		return m_depthImageHeight;
	}

	// get current trajectory transformation
	Eigen::Matrix4f GetTrajectory()
	{
		return m_currentTrajectory;
	}

	bool isCameraMoving()
	{
		return m_isCamMoving;
	}



private:
	bool exists(const std::string& name)
	{
		std::ifstream f(name.c_str());
		return f.good();
	}

	bool readDepthAndColourImageNames(std::string depth_prefix, std::string rgb_prefix, std::string depth_ext, std::string rgb_ext)
	{
		std::stringstream ss;
		int count = 0;
		for (int i = 0; i < m_numFrame + count; i++)
		{
			if (count == 300) break;

			ss.str("");
			ss << depth_prefix << std::setfill('0') << std::setw(6) << i << depth_ext;
			std::string curr_path = m_baseDir + ss.str();
			if (!exists(curr_path))
			{
				std::cout << "No frame " << i << " " << curr_path << std::endl;
				count++;
				continue;
			}
			m_filenameDepthImages.push_back(ss.str());

			ss.str("");
			ss << rgb_prefix << std::setfill('0') << std::setw(6) << i << rgb_ext;
			m_filenameColorImages.push_back(ss.str());

			ss.str("");
			ss << std::setfill('0') << std::setw(6) << i;
			m_ptsFilenames.push_back(ss.str());
		}
		return true;
	}

	bool ReadFileList(const std::string& filename, std::vector<std::string>& result, std::vector<double>& timestamps)
	{
		std::ifstream fileDepthList(filename, std::ios::in);
		if (!fileDepthList.is_open()) return false;
		result.clear();
		timestamps.clear();
		std::string dump;
		std::getline(fileDepthList, dump);
		std::getline(fileDepthList, dump);
		std::getline(fileDepthList, dump);
		while (fileDepthList.good())
		{
			double timestamp;
			fileDepthList >> timestamp;
			std::string filename;
			fileDepthList >> filename;
			if (filename == "") break;
			timestamps.push_back(timestamp);
			result.push_back(filename);
		}
		fileDepthList.close();
		return true;
	}

	bool ReadTrajectoryFile(const std::string& filename, std::vector<Eigen::Matrix4f>& result, std::vector<double>& timestamps)
	{
		std::ifstream file(filename, std::ios::in);
		if (!file.is_open()) return false;
		result.clear();
		std::string dump;
		std::getline(file, dump);
		std::getline(file, dump);
		std::getline(file, dump);

		while (file.good())
		{
			double timestamp;
			file >> timestamp;
			Eigen::Vector3f translation;
			file >> translation.x() >> translation.y() >> translation.z();
			Eigen::Quaternionf rot;
			file >> rot;

			Eigen::Matrix4f transf;
			transf.setIdentity();
			transf.block<3, 3>(0, 0) = rot.toRotationMatrix();
			transf.block<3, 1>(0, 3) = translation;

			if (rot.norm() == 0) break;

			transf = transf.inverse().eval();

			timestamps.push_back(timestamp);
			result.push_back(transf);
		}
		file.close();
		return true;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	// current frame index
	int m_currentIdx;

	int m_increment;

	bool m_isCamMoving;

	int m_numFrame;

	DATASET_TYPE m_dataType;

	// frame data
	float* m_depthFrame;
	BYTE* m_colorFrame;
	Eigen::Matrix4f m_currentTrajectory;
	std::vector<std::vector<int>>* m_landmarks;

	// color camera info
	Eigen::Matrix3f m_colorIntrinsics;
	Eigen::Matrix4f m_colorExtrinsics;
	unsigned int m_colorImageWidth;
	unsigned int m_colorImageHeight;

	// depth (ir) camera info
	Eigen::Matrix3f m_depthIntrinsics;
	Eigen::Matrix4f m_depthExtrinsics;
	unsigned int m_depthImageWidth;
	unsigned int m_depthImageHeight;
	float m_depthScaleFactor;


	// base dir
	std::string m_baseDir;
	// filenamelist depth
	std::vector<std::string> m_filenameDepthImages;
	std::vector<double> m_depthImagesTimeStamps;
	// filenamelist color
	std::vector<std::string> m_filenameColorImages;
	std::vector<double> m_colorImagesTimeStamps;

	std::vector<std::string> m_ptsFilenames;

	// trajectory
	std::vector<Eigen::Matrix4f> m_trajectory;
	std::vector<double> m_trajectoryTimeStamps;
};