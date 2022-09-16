#pragma once
#include "BFMContainer.h"
#include "SimpleMesh.h"


class BFMOptimizer {
private:
    BFMContainer* bfmc;
public:
	BFMOptimizer(BFMContainer* bfmc);
	void optimizeSparseGeometry(const Vertex* faceLandmarks, BFMParameters& parameters, Matrix4d& pose, int landmarksNum);
	void optimizeDenseGeometry(Matrix4d& pose, const std::vector<std::pair<Vertex, Vertex>> correspondences, BFMParameters& parameters);
	void optimizeDenseColour(Matrix4d& pose, const std::vector<std::pair<Vertex, Vertex>> correspondences, BFMParameters& parameters);
};

struct ColorRGBCost {
	ColorRGBCost(BFMContainer* bfmc_, Vector3d meshPointColor_, int bfmIndex_) :
		bfmc{ bfmc_ }, observed_point{ meshPointColor_ }, bfmIndex{ bfmIndex_ } {
		//std::cout << meshPointColor << std::endl;
	}

	template <typename T>
	bool operator()(T const* color_weights, T* residuals) const {

		int length = bfmc->getMeanLength();
		double* colorPCAVariance = bfmc->getColorPCAVariance();
		Map<Matrix<double, Dynamic, Dynamic, RowMajor>> color_pca_basis_full(bfmc->getColorPCABasis(), length, bfmc->getColorLength());
		Matrix<T, 3, 1> vertex_col;
		double* colorMean = bfmc->getColorMean();

		vertex_col[0] = T(colorMean[bfmIndex * 3]);
		vertex_col[1] = T(colorMean[bfmIndex * 3 + 1]);
		vertex_col[2] = T(colorMean[bfmIndex * 3 + 2]);

		for (int i = 0; i < bfmc->getColorLength(); i++) {
			T value = T(sqrt(colorPCAVariance[i])) * color_weights[i];
			vertex_col[0] += T(color_pca_basis_full(bfmIndex * 3, i)) * value;
			vertex_col[1] += T(color_pca_basis_full(bfmIndex * 3 + 1, i)) * value;
			vertex_col[2] += T(color_pca_basis_full(bfmIndex * 3 + 2, i)) * value;
		}

		T meshColor_R = T(observed_point[0]);
		T meshColor_G = T(observed_point[1]);
		T meshColor_B = T(observed_point[2]);

		T r1 = T(meshColor_R - vertex_col[0]);
		T r2 = T(meshColor_G - vertex_col[1]);
		T r3 = T(meshColor_B - vertex_col[2]);

		residuals[0] = T(r1);
		residuals[1] = T(r2);
		residuals[2] = T(r3);

		return true;
	}
private:
	BFMContainer* bfmc;
	Vector3d observed_point;
	int bfmIndex;
};


struct ColorRegularizerCost
{
	ColorRegularizerCost(double weight_)
		: weight{ weight_ }
	{}

	template<typename T>
	bool operator()(T const* color_weights, T* residuals) const
	{

		for (int i = 0; i < COLOR_LENGTH; i++) {
			residuals[i] = color_weights[i] * T(sqrt(weight));
		}
		return true;
	}

private:
	const double weight;
};

struct ShapeRegularizerCost
{
	ShapeRegularizerCost(double weight_)
		: weight{ weight_ }
	{}

	template<typename T>
	bool operator()(T const* shape_weights, T* residuals) const
	{

		for (int i = 0; i < SHAPE_LENGTH; i++) {
			residuals[i] = shape_weights[i] * T(sqrt(weight));
		}
		return true;
	}

private:
	const double weight;
};

struct ExpressionRegularizerCost
{
	ExpressionRegularizerCost(double weight_)
		: weight{weight_}
	{}

	template<typename T>
	bool operator()(T const* exp_weights, T* residuals) const
	{
		for (int j = 0; j < EXPRESSION_LENGTH; j++) {
			residuals[j] = exp_weights[j] * T(sqrt(weight));
		}
		return true;
	}

private:
	const double weight;
};

struct LandmarkMatchCost {
	LandmarkMatchCost(BFMContainer* bfmc_, Vector3d observed_landmark_, int vertex_id_) :
		bfmc{ bfmc_ }, observed_point{ observed_landmark_ }, vertex_id{ vertex_id_ } {}

	template <typename T>
	bool operator()(T const* pose, T const* shape_weights, T const* exp_weights, T* residuals) const {

		int length = bfmc->getMeanLength();
		double* shapePCAVariance = bfmc->getShapePCAVariance();
		double* expressionPCAVariance = bfmc->getExpressionPCAVariance();
		Map<Matrix<double, Dynamic, Dynamic, RowMajor>> shape_pca_basis_full(bfmc->getShapePCABasis(), length, bfmc->getShapeLength());
		Map<Matrix<double, Dynamic, Dynamic, RowMajor>> exp_pca_basis_full(bfmc->getExpressionPCABasis(), length, bfmc->getExpressionLength());

		Matrix<T, 4, 1> vertex_pos;
		double* shapeMean = bfmc->getShapeMean();
		double* expressionMean = bfmc->getExpressionMean();

		vertex_pos[0] = T(shapeMean[vertex_id * 3]) + T(expressionMean[vertex_id * 3]);        // X
		vertex_pos[1] = T(shapeMean[vertex_id * 3 + 1]) + T(expressionMean[vertex_id * 3 + 1]);// Y
		vertex_pos[2] = T(shapeMean[vertex_id * 3 + 2]) + T(expressionMean[vertex_id * 3 + 2]);// Z
		vertex_pos[3] = T(1);

		for (int i = 0; i < bfmc->getShapeLength(); i++) {
			T value = T(sqrt(shapePCAVariance[i])) * shape_weights[i];
			vertex_pos[0] += T(shape_pca_basis_full(vertex_id * 3, i)) * value;
			vertex_pos[1] += T(shape_pca_basis_full(vertex_id * 3 + 1, i)) * value;
			vertex_pos[2] += T(shape_pca_basis_full(vertex_id * 3 + 2, i)) * value;
		}

		for (int i = 0; i < bfmc->getExpressionLength(); i++) {
			T value = T(sqrt(expressionPCAVariance[i])) * exp_weights[i];
			vertex_pos[0] += T(exp_pca_basis_full(vertex_id * 3, i)) * value;
			vertex_pos[1] += T(exp_pca_basis_full(vertex_id * 3 + 1, i)) * value;
			vertex_pos[2] += T(exp_pca_basis_full(vertex_id * 3 + 2, i)) * value;
		}

		Map<const Matrix<T, 4, 4>> poseM (pose);
		Matrix<T, 4, 1> transformed = poseM * vertex_pos;

		T observed_landmark_x = T(observed_point.x());
		T observed_landmark_y = T(observed_point.y());
		T observed_landmark_z = T(observed_point.z());

		// divide cost by scale to make the optimization scale invariant
		T scale = T(poseM(0,0) + poseM(0,1) + poseM(0,2) + poseM(1,0) + poseM(1,1) + poseM(1,2) + poseM(2,0) + poseM(2,1) + poseM(2,2)) / T(3);
		
		residuals[0] = (transformed[0] - observed_landmark_x) / scale;
		residuals[1] = (transformed[1] - observed_landmark_y) / scale;
		residuals[2] = (transformed[2] - observed_landmark_z) / scale;
		
		return true;
	}
private:
	BFMContainer* bfmc;
	Vector3d observed_point;
	int vertex_id;
};

struct MatchCost {
	MatchCost(BFMContainer* bfmc_, Vector3d observed_point_, int vertex_id_) :
		bfmc{ bfmc_ }, observed_point{ observed_point_ }, vertex_id{ vertex_id_ } {}

	template <typename T>
	bool operator()(T const* pose, T const* shape_weights, T const* exp_weights, T* residuals) const {

		int length = bfmc->getMeanLength();
		double* shapePCAVariance = bfmc->getShapePCAVariance();
		double* expressionPCAVariance = bfmc->getExpressionPCAVariance();
		Map<Matrix<double, Dynamic, Dynamic, RowMajor>> shape_pca_basis_full(bfmc->getShapePCABasis(), length, bfmc->getShapeLength());
		Map<Matrix<double, Dynamic, Dynamic, RowMajor>> exp_pca_basis_full(bfmc->getExpressionPCABasis(), length, bfmc->getExpressionLength());

		Matrix<T, 4, 1> vertex_pos;
		double* shapeMean = bfmc->getShapeMean();
		double* expressionMean = bfmc->getExpressionMean();

		vertex_pos[0] = T(shapeMean[vertex_id * 3]) + T(expressionMean[vertex_id * 3]);        // X
		vertex_pos[1] = T(shapeMean[vertex_id * 3 + 1]) + T(expressionMean[vertex_id * 3 + 1]);// Y
		vertex_pos[2] = T(shapeMean[vertex_id * 3 + 2]) + T(expressionMean[vertex_id * 3 + 2]);// Z
		vertex_pos[3] = T(1);

		for (int i = 0; i < bfmc->getShapeLength(); i++) {
			T value = T(sqrt(shapePCAVariance[i])) * shape_weights[i];
			vertex_pos[0] += T(shape_pca_basis_full(vertex_id * 3, i)) * value;
			vertex_pos[1] += T(shape_pca_basis_full(vertex_id * 3 + 1, i)) * value;
			vertex_pos[2] += T(shape_pca_basis_full(vertex_id * 3 + 2, i)) * value;
		}

		for (int i = 0; i < bfmc->getExpressionLength(); i++) {
			T value = T(sqrt(expressionPCAVariance[i])) * exp_weights[i];
			vertex_pos[0] += T(exp_pca_basis_full(vertex_id * 3, i)) * value;
			vertex_pos[1] += T(exp_pca_basis_full(vertex_id * 3 + 1, i)) * value;
			vertex_pos[2] += T(exp_pca_basis_full(vertex_id * 3 + 2, i)) * value;
		}

		Map<const Matrix<T, 4, 4>> poseM(pose);
		Matrix<T, 4, 1> transformed = poseM * vertex_pos;

		T observed_point_x = T(observed_point.x());
		T observed_point_y = T(observed_point.y());
		T observed_point_z = T(observed_point.z());

		// divide cost by scale to make the optimization scale invariant
		T scale = T(poseM(0, 0) + poseM(0, 1) + poseM(0, 2) + poseM(1, 0) + poseM(1, 1) + poseM(1, 2) + poseM(2, 0) + poseM(2, 1) + poseM(2, 2)) / T(3);

		residuals[0] = (transformed[0] - observed_point_x) / scale;
		residuals[1] = (transformed[1] - observed_point_y) / scale;
		residuals[2] = (transformed[2] - observed_point_z) / scale;

		return true;
	}
private:
	BFMContainer* bfmc;
	Vector3d observed_point;
	int vertex_id;
};
