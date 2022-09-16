#pragma once
#include <fstream>
#include <iostream>
#include <vector>
#include <H5Cpp.h>
#include "Eigen.h"
#include "SimpleMesh.h"
#include "common.h"

struct BFMParameters {
	VectorXd shapeWeights;
	VectorXd expressionWeights;
	VectorXd colorWeights;
};

class BFMContainer {
public:
	BFMContainer();
	~BFMContainer();

	std::vector<int> BFMContainer::loadIndexList(std::string filename);
	void loadLandmarks();

	std::vector<int> getLandmarkIndices();
	std::vector<int> getOptimizationSubsetIndices();
	std::vector<Vector3f> getLandmarkCoordinates(std::vector<Vector3d> vertexList);

	void loadH5();
	float* getDataByPath(std::string path);
	double* convertFloatToDouble(float* data, int size);

	int mapFloatToRange(float f);
	double mapIntToDouble(int i);
	void getTriangleIndices();
	void loadAllData();
	void dataCheck();

	BFMParameters generateRandomParams();
	BFMParameters getZeroParams();

	std::vector<Vector3i> getColors(BFMParameters params);
	std::vector<Vector3d> getVertices(BFMParameters params);
	std::vector<Vector3d> getVerticesOfSpecificIds(const std::vector<int> vertexIDs, BFMParameters params);
	std::vector<Vector3d> getVerticesOfMeanFace(const std::vector<int> vertexIDs);

	void writeToObj(std::string filename, BFMParameters params, Matrix4d pose);
	std::vector<Vertex> writeWithBlending(float quotientFromMesh, std::string filename, BFMParameters params, Matrix4d pose, const std::vector<std::pair<Vertex, Vertex>> correspondences);
	std::vector<Vertex> writeDistanceColoured(std::string filename, BFMParameters params, Matrix4d pose, const std::vector<std::pair<Vertex, Vertex>> correspondences);


	double* getColorMean() const { return colorMean; }
	double* getColorPCAVariance() const { return colorPCAVariance; }
	double* getColorPCABasis() const { return colorPCABasis; }

	double* getShapeMean() const { return shapeMean; }
	double* getShapePCAVariance() const { return shapePCAVariance; }
	double* getShapePCABasis() const { return shapePCABasis; }

	double* getExpressionMean() const { return expressionMean; }
	double* getExpressionPCAVariance() const { return expressionPCAVariance; }
	double* getExpressionPCABasis() const { return expressionPCABasis; }

	int getMeanLength() const { return MEAN_LENGTH; }
	int getShapeLength() const { return SHAPE_LENGTH; }
	int getExpressionLength() const { return EXPRESSION_LENGTH; }
	int getColorLength() const { return COLOR_LENGTH; }
private:
    std::string BFM_2019_H5Path = "../Model/BFM/model2019_face12.h5";
	std::string Landmark_ANLPath = "../Model/BFM/our_landmarks68_2019bfm_face12.anl";
	std::string OptimizationSubset_ANLPath = "../Model/BFM/our_fitting_vertices_only.anl";
	// which is deprecated (Version 2009)
    H5::H5File h5_file;

	int* triangles;

	double* colorMean;
	double* colorPCAVariance;
	double* colorPCABasis;

	double* shapeMean;
	double* shapePCAVariance;
	double* shapePCABasis;

	double* expressionMean;
	double* expressionPCAVariance;
	double* expressionPCABasis;
	
	std::vector<int> landmarkVertexIndices, optimizationSubsetVertexIndices;
	std::vector<Vector3f> landmarkCoordinates;

	void distanceToColour(double distance, double& R, double& G, double& B);
};