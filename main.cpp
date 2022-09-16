#include <iostream>
#include <fstream>
#include <array>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <chrono>  

#include "common.h"
#include "BFMContainer.h"
#include "BFMOptimizer.h"
#include "Eigen.h"
#include "MeshFileHelper.h"
#include "PointCloud.h"
#include "ProcrustesAligner.h"
#include "SimpleMesh.h"
#include "VirtualSensor.h"

using namespace std;


std::vector<Eigen::Vector3f> readBFMLandmarks(std::string filename)
{
	std::vector<Eigen::Vector3f> landmarks;
	ifstream fp;
	fp.open(filename);
	if (!fp.is_open())
	{
		std::cout << filename << " not found!" << std::endl;
		return landmarks;
	}

	std::string line;
	while (std::getline(fp, line))
	{
		std::stringstream lineStream(line);
		std::string cell;
		Eigen::Vector3f pt;

		for (int i = 0; std::getline(lineStream, cell, ','); i++)
		{
			pt[i] = std::stof(cell);
			// std::cout << pt[i] << ',';
		}
		// std::cout << std::endl;
		landmarks.push_back(pt);
	}
	fp.close();
	return landmarks;
}

void writeIndices(std::string filename, int* indices)
{
	ofstream fp;
	fp.open(filename);
	if (!fp.is_open())
	{
		std::cout << filename << " not found!" << std::endl;
		return;
	}

	fp << "% 68 Landmark points for Basel Face Model (300W annotation) custom adapted";
	for (int i = 0; i < ALL_LANDMARK_COUNT; i++)
	{ // No newline at the end of the file
		fp << std::endl << indices[i];
	}
	fp.close();
}

void recalculateBFMPointCloud(BFMContainer &bfmc, BFMParameters &params, Matrix4f pose, pcl::PointCloud<pcl::PointXYZ>::Ptr &bfmPointCloud, Vertex* bfmAllPoints) {
	std::cout << "PCLCorrespondenceFinder >> Filling BFM PointCloud." << std::endl;

	std::vector<Vector3d> modelAllPoints = bfmc.getVertices(params);
	std::vector<Vector3i> modelAllPointsColors = bfmc.getColors(params);
	for (int i = 0; i < modelAllPoints.size(); i++) {
		Eigen::Vector4f h_modelPoints = Vector4f(modelAllPoints[i].x(), modelAllPoints[i].y(), modelAllPoints[i].z(), 1.0);
		Eigen::Vector4f transformedSourcePoints = pose * h_modelPoints;

		// fill vertex list
		bfmAllPoints[i].position = Vector4f(transformedSourcePoints.x(), transformedSourcePoints.y(), transformedSourcePoints.z(), 1.0);
		bfmAllPoints[i].color = Vector4uc(modelAllPointsColors[i].x(), modelAllPointsColors[i].y(), modelAllPointsColors[i].z(), 0);

		// fill point cloud
		bfmPointCloud->points[i].x = transformedSourcePoints.x();
		bfmPointCloud->points[i].y = transformedSourcePoints.y();
		bfmPointCloud->points[i].z = transformedSourcePoints.z();
	}
}

void recalculateCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr &meshPointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &bfmPointCloud, Vertex* vertices, Vertex* bfmAllPoints, std::vector<std::pair<Vertex, Vertex>> &correspondenceVertices){
	auto startCorrespondenceFinding = std::chrono::system_clock::now();
	std::cout << "PCLCorrespondenceFinder >> " << "Finding correspondences . . ." << std::endl;

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
	est.setInputSource(bfmPointCloud);
	est.setInputTarget(meshPointCloud);

	pcl::Correspondences allCorrespondences;
	// Determine all reciprocal correspondences
	est.determineCorrespondences(allCorrespondences);

	std::cout << "PCLCorrespondenceFinder >> " << allCorrespondences.size() << " correspondences found." << std::endl;
	auto endCorrespondenceFinding = std::chrono::system_clock::now();
	auto durationCorrespondenceFinding = std::chrono::duration_cast<std::chrono::microseconds>(endCorrespondenceFinding - startCorrespondenceFinding);
	cout << "Timer >> Finding correspondences takes "
		<< double(durationCorrespondenceFinding.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
		<< " seconds." << endl;
	
	
	correspondenceVertices.clear();
	for (int i = 0; i < allCorrespondences.size(); i++) {
		//std::cout << "Point " << allCorrespondences[i].index_query << " in BFM, ";
		//std::cout << "has index of " << allCorrespondences[i].index_match << " in Mesh" << std::endl;
		correspondenceVertices.push_back(
			std::make_pair(
				bfmAllPoints[allCorrespondences[i].index_query],
				vertices[allCorrespondences[i].index_match]
			)
		);
		//std::cout << (int)vertices[allCorrespondences[i].index_match].color[0] << std::endl;
		//std::cout << (int)vertices[allCorrespondences[i].index_match].color[1] << std::endl;
		//std::cout << (int)vertices[allCorrespondences[i].index_match].color[2] << std::endl;
		//vertices[allCorrespondences[i].index_match].color = Vector4uc(0, 255, 0, 0);
	}
}

int main()
{
	// load video

#if (DATASET_FACEGRABBER)
	std::string filenameIn = "../Data/FaceGrabberDB/ID_001/05_Happiness/";
	std::string filenameBaseOut = "face_";
	VirtualSensor sensor(1920, 1080, 512, 424, 1, false, FACE_GRABBER, 30);
#elif (DATASET_KINECT_V2)
	//std::string filenameIn = "../Data/Kinect/26-01-2022_152557/";
	//std::string filenameIn = "../Data/Kinect/02-02-2022_223039/";
	std::string filenameIn = "../Data/Kinect/02-02-2022_223146/";
	std::string filenameBaseOut = "face_";
	VirtualSensor sensor(512, 424, 512, 424, 1, false, KINECT_V2, 30);
#elif (DATASET_REALSENSE)
	std::string filenameIn = "../Data/RealSense/Daniel3-angles/";
	std::string filenameBaseOut = "face_";
	VirtualSensor sensor(1280, 720, 1280, 720, 1, false, REALSENSE, 30);
#else
	std::string filenameIn = "../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";
	VirtualSensor sensor;
#endif

	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path! " << filenameIn << std::endl;
		return -1;
	}
	auto startOpenCVInit = std::chrono::system_clock::now();
#if (DATASET_KINECT_V2)
	OpenCVImgCropper ic = OpenCVImgCropper(512, 424);
#else 
	OpenCVImgCropper ic = OpenCVImgCropper(640, 360);
#endif
	auto endOpenCVInit = std::chrono::system_clock::now();
	auto durationOpenCVInit = std::chrono::duration_cast<std::chrono::microseconds>(endOpenCVInit - startOpenCVInit);
	cout << "Timer >> OpenCV Initialization takes "
		<< double(durationOpenCVInit.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
		<< " seconds." << endl;
	LandmarkDetector ld = LandmarkDetector();
	std::stringstream ss;

	auto startLoadH5 = std::chrono::system_clock::now();
	// Load the Base Face Model
	BFMContainer bfmc = BFMContainer();
	bfmc.loadH5();
	bfmc.loadLandmarks();
	bfmc.loadAllData();
	//bfmc.dataCheck();

	std::vector<int> BFMLandmarkIndices = bfmc.getLandmarkIndices();
	std::vector<Vector3d> BFMLandmarksDouble = bfmc.getVerticesOfMeanFace(BFMLandmarkIndices);
	std::vector<Vector3f> BFMLandmarks;
	for (Vector3d p : BFMLandmarksDouble)
		BFMLandmarks.push_back(p.cast<float>());

	std::cout << "Landmarks loaded from model " << BFMLandmarksDouble.size() << " / " << BFMLandmarks.size() << std::endl;
	ASSERT(BFMLandmarks.size() == LANDMARK_COUNT && "Wrong number of landmarks loaded");
	auto endLoadH5 = std::chrono::system_clock::now();
	auto durationLoadH5 = std::chrono::duration_cast<std::chrono::microseconds>(endLoadH5 - startLoadH5);
	cout << "Timer >> Loading H5 takes "
		<< double(durationLoadH5.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
		<< " seconds." << endl;

#if (REMOVE_OUTLINE_LANDMARKS)
	BFMLandmarkIndices.erase(BFMLandmarkIndices.begin(), BFMLandmarkIndices.begin() + 17);
#endif


	// handle input frame by frame
	while (sensor.ProcessNextFrame())
	{
		ss.str("");
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt();
		std::string currentFrameBasename = ss.str();

		auto startProcessFrame = std::chrono::system_clock::now();
		auto startLoadImage = std::chrono::system_clock::now();

		float* depthMap = sensor.GetDepth();
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);
		Matrix4f intrinsics(4, 4);
#if DATASET_REALSENSE
		fX = 904.0526733398438;
		cX = 643.0077514648438;
		fY = 904.7403564453125;
		cY = 352.6210632324219;
#endif
		intrinsics <<
			fX, 0, cX, 0,
			0, fY, cY, 0,
			0,  0,  1, 0,
			0,  0,  0, 1;
		Matrix4f intrinsicsInv = intrinsics.inverse();


		// compute inverse of depth extrinsics (camera position)
		Eigen::Matrix4f extrinsics, extrinsicsInv;
		if (sensor.isCameraMoving())
		{
			extrinsics = sensor.GetTrajectory();
			extrinsicsInv = sensor.GetTrajectory().inverse();
		}
		else
		{
			extrinsics.setIdentity();
			extrinsicsInv = extrinsics.inverse();
		}

		// sourcePoints is BFM landmarks array
		std::vector<Vector3f> sourcePoints = BFMLandmarks;
		std::vector<Vector3f> targetPoints;

		// back-projection
		int height = (int)sensor.GetDepthImageHeight();
		int width = (int)sensor.GetDepthImageWidth();

		// vertices is point clouds
		Vertex* vertices = new Vertex[height * width];
		int idx = 0;
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				idx = i * width + j;
				if (depthMap[idx] == MINF)
				{
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				}
				else
				{
					Vector4f h_campt = Vector4f(j * depthMap[idx], i * depthMap[idx], depthMap[idx], 1.0);
					Vector4f worldpt = extrinsicsInv * intrinsicsInv * h_campt;
					vertices[idx].position = worldpt;
					vertices[idx].color = Vector4uc(colorMap[4 * idx], colorMap[4 * idx + 1], colorMap[4 * idx + 2], colorMap[4 * idx + 3]);
				}
			}
		}

		auto endtLoadImage = std::chrono::system_clock::now();
		auto durationLoadImage = std::chrono::duration_cast<std::chrono::microseconds>(endtLoadImage - startLoadImage);
		cout << "Timer >> Load image takes "
			<< double(durationLoadImage.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
			<< " seconds." << endl;
		// Detect Landmarks in 2D
		// lm_vertices is 3D Vertex array achieved by Dlib and projection
		Vertex* lm_vertices = new Vertex[LANDMARK_COUNT];
		Vertex* lm_allVertices = new Vertex[LANDMARK_COUNT];
		// bfm_vertices is 3D Vertex array achieved by BFM Model
		Vertex* bfm_lm_vertices = new Vertex[LANDMARK_COUNT];

		auto startOpenCVDetect = std::chrono::system_clock::now();

		// Use OpenCV to detect face and crop to get facial area
		Tuple face_area_comp;
		try {
			face_area_comp = ic.getMatOfFace(sensor.GetColorImageHeight(), sensor.GetColorImageWidth(), colorMap);
		}
		catch (cv::Exception & e) {
			// output exception message
			cerr << e.msg << endl;
		}
		auto endOpenCVDetect = std::chrono::system_clock::now();
		auto durationOpenCVDetect = std::chrono::duration_cast<std::chrono::microseconds>(endOpenCVDetect - startOpenCVDetect);
		cout << "Timer >> OpenCV Face detection takes "
			<< double(durationOpenCVDetect.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
			<< " seconds." << endl;

		// 2D Landmarks landmarks[0] -> left chin first point
		// landmarks[0][0] = landmarks[0].x
		auto startLandmarkDetect = std::chrono::system_clock::now();

		ss.str("");
		ss << std::setfill('0') << std::setw(6) << sensor.GetCurrentFrameCnt();
		std::vector<std::vector<int>> landmarks_2D = ld.detect(face_area_comp, ss.str());

		auto endLandmarkDetect = std::chrono::system_clock::now();
		auto durationLandmarkDetect = std::chrono::duration_cast<std::chrono::microseconds>(endLandmarkDetect - startLandmarkDetect);
		cout << "Timer >> DLib landmark detection takes "
			<< double(durationLandmarkDetect.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
			<< " seconds." << endl;

		if(landmarks_2D.size() <= 0){
			cout << "No face detected! Skipping Frame" << std::endl;
			continue;
		}
		
#if (REMOVE_OUTLINE_LANDMARKS)
		landmarks_2D.erase(landmarks_2D.begin(), landmarks_2D.begin() + 17); // Remove outline of the face (landmarks 1 - 17)
		sourcePoints.erase(sourcePoints.begin(), sourcePoints.begin() + 17);
#endif
		std::cout << "Num of Landmarks: " << landmarks_2D.size() << " / " << sourcePoints.size() << std::endl;

		
		float faceMeanZValue, faceMaxZValueDifference;
		{
			// Filter out defective landmarks:
			// * Find mean 2d location of landmarks ("middle of the face")
			// * Calculate the average z-depth value of this area (+- a few pixels)
			// * Calculate the expected z-scale of the face (max the same as the x-scale)
			// * Throw out any landmarks where the z-value deviates more than that

			Vector2f faceMeanImagePos = Vector2f::Zero(), faceVarImagePos = Vector2f::Zero();
			for (int i = 0; i < landmarks_2D.size(); i++)
			{
				Vector2f lPos = Vector2f(landmarks_2D[i][0], landmarks_2D[i][1]);
				// std::cout << "Landmark " << i << " coordinates: " << lPos;
				faceMeanImagePos += lPos;
				faceVarImagePos += Vector2f(landmarks_2D[i][0] * landmarks_2D[i][0], landmarks_2D[i][1] * landmarks_2D[i][1]);
			}
			faceMeanImagePos /= landmarks_2D.size();
			faceVarImagePos /= landmarks_2D.size();
			faceVarImagePos -= Vector2f(faceMeanImagePos[0]*faceMeanImagePos[0], faceMeanImagePos[1]*faceMeanImagePos[1]);
			std::cout << "Landmark mean coordinates: " << faceMeanImagePos << " Std.Dev: " << faceVarImagePos.cwiseSqrt() << std::endl;

			Vector2i faceMeanImagePosI = faceMeanImagePos.cast<int>();
			faceMeanZValue = 0;
			float faceVarZValue = 0;
			const int sampleGridNum = 5, sampleGridOffset = sampleGridNum/2;
			Vector2f sampleWidth = faceVarImagePos.cwiseSqrt()*1.0 / sampleGridNum; // sample -0.5std.dev | +0.5std.dev
			std::vector<float> samples;
			for(int i = 0; i < sampleGridNum; i++){
				for(int j = 0; j < sampleGridNum; j++){
					// get Index within the 2D Frame
					int px = faceMeanImagePosI.x() + (i - sampleGridOffset) * sampleWidth.x();
					int py = faceMeanImagePosI.y() + (j - sampleGridOffset) * sampleWidth.y();
					idx = py * width + px;
					float z = depthMap[idx];
					std::cout << "Sampling face at " << px << " , " << py << " : " << z << std::endl;
					if (z != MINF){ // ignore all samples that are not detected properly
						samples.push_back(z);
					}
				}
			}
			faceMeanZValue = std::accumulate(samples.begin(), samples.end(), 0.0);
			faceVarZValue = std::inner_product(samples.begin(), samples.end(), samples.begin(), 0.0);
			faceMeanZValue /= samples.size();
			faceVarZValue /= samples.size();
			// std::cout << "E[z^2] " << faceVarZValue << std::endl;
			faceVarZValue -= faceMeanZValue * faceMeanZValue;
			//std::cout << "Var[z] " << faceVarZValue << std::endl;
			float faceStdDevZValue = sqrt(faceVarZValue);

			int sampleCountBefore = samples.size();
			// throw out all samples that are > 2 std.dev away from mean and repeat calculation
			samples.erase(std::remove_if(samples.begin(), samples.end(), [&](const float& z) {
				return abs(z - faceMeanZValue) > faceStdDevZValue * 2;
			}), samples.end());
			std::cout << "Filtered out " << (sampleCountBefore - samples.size()) << " samples, " << samples.size() << " remaining" << std::endl;
			faceMeanZValue = std::accumulate(samples.begin(), samples.end(), 0.0);
			faceVarZValue = std::inner_product(samples.begin(), samples.end(), samples.begin(), 0.0);
			faceMeanZValue /= samples.size();
			faceVarZValue /= samples.size();
			// std::cout << "E[z^2] " << faceVarZValue << std::endl;
			faceVarZValue -= faceMeanZValue * faceMeanZValue;
			//std::cout << "Var[z] " << faceVarZValue << std::endl;

			float faceAvgSize = sqrt(faceVarImagePos.mean())*4; // Total face size is typically 4 times the std deviation of the landmarks
			float cameraAvgScaleFactor = intrinsicsInv.block(0,0,2,2).mean()*2;
			std::cout << "Avg Face Size px: " << faceAvgSize << std::endl;
			std::cout << "Avg Camera Scale factor: " << cameraAvgScaleFactor << std::endl;
			// Maximal Z Value difference allowed = avg. width of face in world space
			faceMaxZValueDifference = cameraAvgScaleFactor * faceAvgSize * faceMeanZValue;
			std::cout << "Face mean z: " << faceMeanZValue << " Std.Dev: " << faceStdDevZValue << std::endl;
			std::cout << "Maximal Face Z range allowed: " << faceMaxZValueDifference << std::endl;
		}

		int ignoredLandmarks = 0;
		for (int i = 0; i < landmarks_2D.size(); i++)
		{
			// get Index within the 2D Frame
			idx = landmarks_2D[i][1] * width + landmarks_2D[i][0];
#if (DISPLAY_LANDMARK_ON_FACE)
			// here draw a red cross on face landmarks
			// Idx-w-1     Idx-w-2      Idx-w     Idx-w+1    Idx-w+2
			//    -1         -2          Idx         +1        +2
			// Idx+w-1     Idx+w-2      Idx+w     Idx+w+1    Idx+w+2
			if (depthMap[idx] != MINF)
			{
				vertices[idx].color = Vector4uc(255, 0, 0, 0);
				vertices[idx - 2].color = Vector4uc(255, 0, 0, 0);
				vertices[idx - 1].color = Vector4uc(255, 0, 0, 0);
				vertices[idx + 1].color = Vector4uc(255, 0, 0, 0);
				vertices[idx + 2].color = Vector4uc(255, 0, 0, 0);

				vertices[idx - 2 - width].color = Vector4uc(255, 0, 0, 0);
				vertices[idx - 1 - width].color = Vector4uc(255, 0, 0, 0);
				vertices[idx + 1 - width].color = Vector4uc(255, 0, 0, 0);
				vertices[idx + 2 - width].color = Vector4uc(255, 0, 0, 0);

				vertices[idx - 2 + width].color = Vector4uc(255, 0, 0, 0);
				vertices[idx - 1 + width].color = Vector4uc(255, 0, 0, 0);
				vertices[idx + 1 + width].color = Vector4uc(255, 0, 0, 0);
				vertices[idx + 2 + width].color = Vector4uc(255, 0, 0, 0);

				vertices[idx - 2 - 2 * width].color = Vector4uc(255, 0, 0, 0);
				vertices[idx + 2 - 2 * width].color = Vector4uc(255, 0, 0, 0);

				vertices[idx - 2 + 2 * width].color = Vector4uc(255, 0, 0, 0);
				vertices[idx + 2 + 2 * width].color = Vector4uc(255, 0, 0, 0);
			}
#endif
			
			if (depthMap[idx] == MINF)
			{
				std::cout << "Error, landmark " << i << " could not be placed in 3D space!" << std::endl;
				lm_vertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
				lm_vertices[i].color = Vector4uc(0, 0, 0, 0);
				lm_allVertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
				lm_allVertices[i].color = Vector4uc(0, 0, 0, 0);
				sourcePoints.erase(sourcePoints.begin() + (i - ignoredLandmarks));
				ignoredLandmarks++;
			}
			else if(abs(depthMap[idx]-faceMeanZValue) > faceMaxZValueDifference){
				std::cout << "Error, landmark " << i << " ignored because its z-value is too far (" << abs(depthMap[idx]-faceMeanZValue) << ") out of the expected range!" << std::endl;
				
				float x = landmarks_2D[i][0] * depthMap[idx];
				float y = landmarks_2D[i][1] * depthMap[idx];
				float z = depthMap[idx];
				Vector4f h_campt = Vector4f(x, y, z, 1.0);
				Vector4f worldpt = extrinsicsInv * intrinsicsInv * h_campt;
				lm_allVertices[i].position = worldpt;
				lm_allVertices[i].color = Vector4uc(255,0,0,255);

				lm_vertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
				lm_vertices[i].color = Vector4uc(0, 0, 0, 0);
				sourcePoints.erase(sourcePoints.begin() + (i - ignoredLandmarks));
				ignoredLandmarks++;
			}
			else
			{
				float x = landmarks_2D[i][0] * depthMap[idx];
				float y = landmarks_2D[i][1] * depthMap[idx];
				float z = depthMap[idx];
				Vector4f h_campt = Vector4f(x, y, z, 1.0);
				Vector4f worldpt = extrinsicsInv * intrinsicsInv * h_campt;
				targetPoints.push_back(Vector3f(worldpt.x(), worldpt.y(), worldpt.z()));
				lm_vertices[i].position = worldpt;
				// Get original colour
				lm_vertices[i].color = Vector4uc(colorMap[4 * idx], colorMap[4 * idx + 1], colorMap[4 * idx + 2], colorMap[4 * idx + 3]);
				lm_allVertices[i].position = worldpt;
				lm_allVertices[i].color = Vector4uc(0,255,0,255);
			}
		}
		// write mesh file
#if (SAVE_SCENE_MESH)
		WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), currentFrameBasename+".off");
#endif
#if (SAVE_3D_LANDMARKS_MESH)
		WriteLandmarks(lm_allVertices, landmarks_2D.size(), currentFrameBasename+"_landmark.off");
#endif


		int goodLandmarksLeft = landmarks_2D.size() - ignoredLandmarks;
		std::cout << "Ignored " << ignoredLandmarks << " Landmarks; " << goodLandmarksLeft << " good landmarks left" << std::endl;
		if(goodLandmarksLeft < 10){
			cout << "Too many landmarks defective! Only " << goodLandmarksLeft << " landmarks left. Skipping Frame" << std::endl;
			continue;
		}




		// ~~~~   Face Optimization   ~~~~

		Matrix4f estimatedPose;
		Matrix4d estimatedPoseD;
		BFMParameters params = bfmc.getZeroParams();
		Vertex* bfmAllPoints = new Vertex[BFM_VERTEX_COUNT];
		pcl::PointCloud<pcl::PointXYZ>::Ptr bfmPointCloud(new pcl::PointCloud<pcl::PointXYZ>(BFM_VERTEX_COUNT, 1));
		pcl::PointCloud<pcl::PointXYZ>::Ptr meshPointCloud(new pcl::PointCloud<pcl::PointXYZ>(width * height, 1));
		// pair of BFM, Mesh !!! first is BFM Vertex second is Mesh Vertex
		std::vector<std::pair<Vertex, Vertex>> correspondenceVertices;
		

		// PROCRUSTES
		{
			auto startProcrustes = std::chrono::system_clock::now();
			// Estimate the pose from source to target mesh with Procrustes alignment.
			ProcrustesAligner aligner;
			std::cout << "Start to estimated pose..." << std::endl;
			estimatedPose = aligner.estimatePose(sourcePoints, targetPoints);
			std::cout << "Estimated Pose: " << std::endl << estimatedPose << std::endl;
			auto endProcrustes = std::chrono::system_clock::now();
			auto durationProcrustes = std::chrono::duration_cast<std::chrono::microseconds>(endProcrustes - startProcrustes);
			cout << "Timer >> Estimate Procrustes takes "
				<< double(durationProcrustes.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
				<< " seconds." << endl;
		}



		// ICP & Point Cloud setup
		{
			std::cout << "------ Searching Correspondence -----" << std::endl;

			std::cout << "PCLCorrespondenceFinder >> Filling Depth Image PointCloud." << std::endl;
			auto startPCLInit = std::chrono::system_clock::now();
			for (int i = 0; i < height * width; i++) {
				meshPointCloud->points[i].x = vertices[i].position.x();
				meshPointCloud->points[i].y = vertices[i].position.y();
				meshPointCloud->points[i].z = vertices[i].position.z();
			}
			recalculateBFMPointCloud(bfmc, params, estimatedPose, bfmPointCloud, bfmAllPoints);

			auto endPCLInit = std::chrono::system_clock::now();
			auto durationPCLInit = std::chrono::duration_cast<std::chrono::microseconds>(endPCLInit - startPCLInit);
			cout << "Timer >> PCL Initialization takes "
				<< double(durationPCLInit.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
				<< " seconds." << endl;
			std::cout << "PCLCorrespondenceFinder >> " << "BFM has " << bfmPointCloud->size() << " data points." << std::endl;
			std::cout << "PCLCorrespondenceFinder >> " << "Mesh has " << meshPointCloud->size() << " data points." << std::endl;

			auto startICP = std::chrono::system_clock::now();

			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			icp.setInputSource(bfmPointCloud);
			icp.setInputTarget(meshPointCloud);

			std::cout << "PCLCorrespondenceFinder >> " << "Performing ICP Registration . . ." << std::endl;
			pcl::PointCloud<pcl::PointXYZ> icpAligned;
			icp.align(icpAligned);
			std::cout << "PCLCorrespondenceFinder >> " << "ICP converged with transformation: " << std::endl;
			Matrix4f icpTransform = icp.getFinalTransformation();
			auto endICP = std::chrono::system_clock::now();
			auto durationICP = std::chrono::duration_cast<std::chrono::microseconds>(endICP - startICP);
			cout << "Timer >> PCL Initialization takes "
				<< double(durationICP.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
				<< " seconds." << endl;
			std::cout << icpTransform << std::endl;

			// Update the estimated pose
			estimatedPose = icpTransform * estimatedPose;
		}
		
		estimatedPoseD = estimatedPose.cast<double>();

		BFMOptimizer bfmOptimizer = BFMOptimizer(&bfmc);
		std::cout << "ShapeParams before Optimization: " << std::endl << params.shapeWeights[0] << std::endl;
		std::cout << "ColorParams before Optimization: " << std::endl << params.colorWeights[0] << std::endl;
		std::cout << "ExpressionParams before Optimization: " << std::endl << params.expressionWeights[0] << std::endl;


		recalculateBFMPointCloud(bfmc, params, estimatedPose, bfmPointCloud, bfmAllPoints);
		recalculateCorrespondences(meshPointCloud, bfmPointCloud, vertices, bfmAllPoints, correspondenceVertices);
		bfmc.writeDistanceColoured(currentFrameBasename+"_face_distance_1_procrustes", params, estimatedPoseD, correspondenceVertices);


		// Sparse (Landmark matching) optimization
		{
			auto startSparseOptimizing = std::chrono::system_clock::now();
			
			bfmOptimizer.optimizeSparseGeometry(lm_vertices, params, estimatedPoseD, targetPoints.size());
			
			auto endSparseOptimizing = std::chrono::system_clock::now();
			auto durationSparseOptimizing = std::chrono::duration_cast<std::chrono::microseconds>(endSparseOptimizing - startSparseOptimizing);
			cout << "Timer >> Optimizing sparse term takes "
				<< double(durationSparseOptimizing.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
				<< " seconds." << endl;
			
			// Redo the matching of the vertices to points
			recalculateBFMPointCloud(bfmc, params, estimatedPose, bfmPointCloud, bfmAllPoints);
			recalculateCorrespondences(meshPointCloud, bfmPointCloud, vertices, bfmAllPoints, correspondenceVertices);
			bfmc.writeDistanceColoured(currentFrameBasename+"_face_distance_2_sparse", params, estimatedPoseD, correspondenceVertices);
		}


		// Dense (closest points) optimization
		for(int i = 0; i < DENSE_GEOMETRY_ITERATIONS; i++){
			auto startDenseGeometryOptimizing = std::chrono::system_clock::now();
			
			bfmOptimizer.optimizeDenseGeometry(estimatedPoseD, correspondenceVertices, params);
			
			auto endDenseGeometryOptimizing = std::chrono::system_clock::now();
			auto durationDenseGeometryOptimizing = std::chrono::duration_cast<std::chrono::microseconds>(endDenseGeometryOptimizing - startDenseGeometryOptimizing);
			cout << "Timer >> Optimizing dense geometry term takes "
				<< double(durationDenseGeometryOptimizing.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
				<< " seconds." << endl;
			
			estimatedPose = estimatedPoseD.cast<float>();
			recalculateBFMPointCloud(bfmc, params, estimatedPose, bfmPointCloud, bfmAllPoints);
			recalculateCorrespondences(meshPointCloud, bfmPointCloud, vertices, bfmAllPoints, correspondenceVertices);
			bfmc.writeDistanceColoured(currentFrameBasename+"_face_distance_3_"+to_string(i)+"_final", params, estimatedPoseD, correspondenceVertices);
		}

		

		// Dense Color Optimization
		{
			auto startDenseColorOptimizing = std::chrono::system_clock::now();

			bfmOptimizer.optimizeDenseColour(estimatedPoseD, correspondenceVertices, params);

			auto endDenseColorOptimizing = std::chrono::system_clock::now();
			auto durationDenseColorOptimizing = std::chrono::duration_cast<std::chrono::microseconds>(endDenseColorOptimizing - startDenseColorOptimizing);
			cout << "Timer >> Optimizing dense color term takes "
				<< double(durationDenseColorOptimizing.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
				<< " seconds." << endl;
		}

		std::cout << "Params after Optimization: " << std::endl << params.shapeWeights[0] << std::endl;
		std::cout << "ColorParams after Optimization: " << std::endl << params.colorWeights[0] << std::endl;
		std::cout << "ExpressionParams after Optimization: " << std::endl << params.expressionWeights[0] << std::endl;



		auto startOutput = std::chrono::system_clock::now();

#if (SAVE_3D_LANDMARKS_MESH)
		std::vector<Vector3d> modelLandmarkPoints = bfmc.getVerticesOfSpecificIds(BFMLandmarkIndices, params);
		for (int i = 0; i < modelLandmarkPoints.size(); i++)
		{
			Eigen::Vector4f h_modelPoints = Vector4f(modelLandmarkPoints[i].x(), modelLandmarkPoints[i].y(), modelLandmarkPoints[i].z(), 1.0);
			Eigen::Vector4f transformedSourcePoints = estimatedPose * h_modelPoints;
			bfm_lm_vertices[i].position = Vector4f(transformedSourcePoints.x(), transformedSourcePoints.y(), transformedSourcePoints.z(), 1.0);
			bfm_lm_vertices[i].color = Vector4uc(255, 255, 255, 255);
		}
		WriteLandmarks(bfm_lm_vertices, sourcePoints.size(), currentFrameBasename+"_landmark_bfm.off");
#endif

#if (RENDER_COLOR)
		std::vector<Vertex> finalVertices = bfmc.writeWithBlending(COLOR_BLENDING_WEIGHT, currentFrameBasename+"_face", params, estimatedPoseD, correspondenceVertices);
#else
		std::vector<Vertex> finalVertices = bfmc.writeWithBlending(0.0, currentFrameBasename+"_face", params, estimatedPoseD, correspondenceVertices);
#endif

		bfmc.writeToObj(currentFrameBasename+"_face_params_only", params, Matrix4d::Identity());
		//bfmc.writeToObj(currentFrameBasename+"_face_transform_only", bfmc.generateRandomParams(), estimatedPoseD);

#if (SAVE_FINAL_REPROJECTION)
		cv::Mat oImg = ic.getOriginalImg(sensor.GetColorImagePath());
		//std::cout << oImg.rows << std::endl;
		//std::cout << oImg.cols << std::endl;
		std::vector<std::vector<float>> zBuffer;
		zBuffer.assign(height, std::vector<float>(width, 1.));

		for (Vertex fv : finalVertices) {
			//std::cout << "z: " << fv.position.z() << std::endl;
			int x = ceil(((fv.position.x() * fX) / fv.position.z()) + cX);
			int y = ceil(((fv.position.y() * fY) / fv.position.z()) + cY);
			//std::cout << "Buf " << zBuffer.at(y).at(x) << std::endl;

			if (fv.position.z() < zBuffer.at(y).at(x)) {
				zBuffer.at(y).at(x) = fv.position.z();
				oImg.at<cv::Vec3b>(y, x) = cv::Vec3b((int)fv.color.z(), (int)fv.color.y(), (int)fv.color.x());
			}
			else {
				continue;
			}
			//std::cout << "x " << x << "  y " << y << std::endl;
		}
		ss.str("");
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << "_reprojected.png";
		cv::imwrite(ss.str(), oImg);
#if (DISPLAY_FINAL_REPROJECTION)
		cv::imshow("img", oImg);
		cv::waitKey(0);
#endif
#endif
		auto endOutput = std::chrono::system_clock::now();
		auto durationOutput = std::chrono::duration_cast<std::chrono::microseconds>(endOutput - startOutput);
		cout << "Timer >> Output takes "
			<< double(durationOutput.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
			<< " seconds." << endl;

		delete[] vertices;
		auto endProcessFrame = std::chrono::system_clock::now();
		auto durationOneFrame = std::chrono::duration_cast<std::chrono::microseconds>(endProcessFrame - startProcessFrame);
		cout << "Timer >> Process one frame takes "
			<< double(durationOneFrame.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
			<< " seconds." << endl;
	}

	return 0;
}