#include "BFMOptimizer.h"
#include "common.h"
#include <ceres/ceres.h>

ceres::Solver::Options options;

BFMOptimizer::BFMOptimizer(BFMContainer* bfmc) : bfmc(bfmc) {
	std::cout << "----- BFMOptimizer Init Start -----" << std::endl;
	options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.num_threads = 16;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = MAX_NUM_CERES_ITERATIONS;
}

void BFMOptimizer::optimizeSparseGeometry(const Vertex* faceLandmarks, BFMParameters& parameters, Matrix4d& pose, int landmarksNum){
	ceres::Problem optimizationSparseProblem;
	const int shapeLength = bfmc->getShapeLength();
	const int expressionLength = bfmc->getExpressionLength();

	int numConstraints = 0;
	for (int j = 0; j < landmarksNum; j++) {
		if (faceLandmarks[j].position[0] == MINF)
		{
			std::cout << "Do not add residual block for landmark[" << j << "]" << std::endl;
			continue;
		}
		Vertex faceLandmark = faceLandmarks[j];
		Vector3d faceLandmark_3d = { (double)faceLandmark.position.x(), (double)faceLandmark.position.y(), (double)faceLandmark.position.z()};

#if (REMOVE_OUTLINE_LANDMARKS)
		ceres::CostFunction* landmarkCost = new ceres::AutoDiffCostFunction<LandmarkMatchCost, 3, 16, SHAPE_LENGTH, EXPRESSION_LENGTH>(
			new LandmarkMatchCost(bfmc, faceLandmark_3d, bfmc->getLandmarkIndices()[j + OUTLINE_LANDMARK_COUNT])
			);
#else
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<LandmarkMatchCost, 3, 16, SHAPE_LENGTH, EXPRESSION_LENGTH>(
			new LandmarkMatchCost(bfmc, faceLandmark_3d, bfmc->getLandmarkIndices()[j])
			);
#endif
		optimizationSparseProblem.AddResidualBlock(landmarkCost, NULL, pose.data(), parameters.shapeWeights.data(), parameters.expressionWeights.data());
		numConstraints++;
	}

	// Don't optimize the Transformation Matrix
	optimizationSparseProblem.SetParameterBlockConstant(pose.data());
	

	ceres::CostFunction* shapeCost = new ceres::AutoDiffCostFunction<ShapeRegularizerCost, SHAPE_LENGTH, SHAPE_LENGTH>(
		new ShapeRegularizerCost(SHAPE_REGULARIZATION_WEIGHT * numConstraints)
		);
	optimizationSparseProblem.AddResidualBlock(shapeCost, NULL, parameters.shapeWeights.data());

	ceres::CostFunction* expressionCost = new ceres::AutoDiffCostFunction<ExpressionRegularizerCost, EXPRESSION_LENGTH, EXPRESSION_LENGTH>(
		new ExpressionRegularizerCost(EXPRESSION_REGULARIZATION_WEIGHT * numConstraints)
		);
	optimizationSparseProblem.AddResidualBlock(expressionCost, NULL, parameters.expressionWeights.data());


	ceres::Solver::Summary summary;
	ceres::Solve(options, &optimizationSparseProblem, &summary);
	std::cout << "BFMOptimizer >> solving SparseTerm . . ." << std::endl;
	std::cout << summary.BriefReport() << std::endl;
	std::cout << "BFMOptimizer >> SparseTerm solved.\n" << std::endl;
}



void BFMOptimizer::optimizeDenseGeometry(Matrix4d& pose, const std::vector<std::pair<Vertex, Vertex>> correspondences, BFMParameters& parameters) {
	ceres::Problem optimizationDenseProblem;
	std::vector<int> vertexIDs = bfmc->getOptimizationSubsetIndices();
	std::cout << "Number of correspondences: " << vertexIDs.size() << std::endl;

	int c = std::rand() % OPTIMIZE_DENSE_GEOMETRY_REDUCE_BY; // random offset so repeated optimization targets other vertices
	const int reduction = OPTIMIZE_DENSE_GEOMETRY_REDUCE_BY;
	int numConstraints = 0;
	for (int j : vertexIDs) {
		c++;
		if (c % reduction != 0)
			continue;
		Vertex meshPoint = correspondences[j].second;
		Vector4uc observed_point = meshPoint.color;

		Vector3d meshPointPosition_double = Vector3d{ (double)meshPoint.position.x(), (double)meshPoint.position.y(), (double)meshPoint.position.z() };

		Vector3d meshPointColor_double = Vector3d{ bfmc->mapIntToDouble((int)observed_point.x()),
												   bfmc->mapIntToDouble((int)observed_point.y()),
												   bfmc->mapIntToDouble((int)observed_point.z()) };

		ceres::CostFunction* matchCost = new ceres::AutoDiffCostFunction<MatchCost, 3, 16, SHAPE_LENGTH, EXPRESSION_LENGTH>(
			new MatchCost(bfmc, meshPointPosition_double, j)
			);
		optimizationDenseProblem.AddResidualBlock(matchCost, NULL, pose.data(), parameters.shapeWeights.data(), parameters.expressionWeights.data());
		numConstraints++;
	}

	ceres::CostFunction* shapeCost = new ceres::AutoDiffCostFunction<ShapeRegularizerCost, SHAPE_LENGTH, SHAPE_LENGTH>(
		new ShapeRegularizerCost(SHAPE_REGULARIZATION_WEIGHT * numConstraints)
		);
	optimizationDenseProblem.AddResidualBlock(shapeCost, NULL, parameters.shapeWeights.data());

	ceres::CostFunction* expressionCost = new ceres::AutoDiffCostFunction<ExpressionRegularizerCost, EXPRESSION_LENGTH, EXPRESSION_LENGTH>(
		new ExpressionRegularizerCost(EXPRESSION_REGULARIZATION_WEIGHT * numConstraints)
		);
	optimizationDenseProblem.AddResidualBlock(expressionCost, NULL, parameters.expressionWeights.data());


	// Don't optimize the Transformation Matrix
	optimizationDenseProblem.SetParameterBlockConstant(pose.data());

	ceres::Solver::Summary summary;
	std::cout << "BFMOptimizer >> solving DenseGeometryTerm . . ." << std::endl;
	ceres::Solve(options, &optimizationDenseProblem, &summary);
	std::cout << summary.BriefReport() << std::endl;
	std::cout << "BFMOptimizer >> DenseGeometryTerm solved.\n" << std::endl;
}



void BFMOptimizer::optimizeDenseColour(Matrix4d & pose, const std::vector<std::pair<Vertex, Vertex>> correspondences, BFMParameters & parameters) {
	ceres::Problem optimizationDenseProblem;
	std::cout << "Number of correspondences: " << correspondences.size() << std::endl;
	const int reduction = OPTIMIZE_DENSE_COLOR_REDUCE_BY;
	int numConstraints = 0;
	for (int j = 0; j < correspondences.size(); j += reduction) {
		Vertex meshPoint = correspondences[j].second;
		Vector4uc observed_point = meshPoint.color;

		//Vector3d meshPointPosition_double = Vector3d{ (double)meshPoint.position.x(), (double)meshPoint.position.y(), (double)meshPoint.position.z() };

		Vector3d meshPointColor_double = Vector3d{ bfmc->mapIntToDouble((int)observed_point.x()),
												   bfmc->mapIntToDouble((int)observed_point.y()),
												   bfmc->mapIntToDouble((int)observed_point.z()) };
		ceres::CostFunction* colorCost = new ceres::AutoDiffCostFunction<ColorRGBCost, 3, COLOR_LENGTH>(
			new ColorRGBCost(bfmc, meshPointColor_double, j)
			);
		optimizationDenseProblem.AddResidualBlock(colorCost, NULL, parameters.colorWeights.data());
		numConstraints++;
	}
	std::cout << "Added " << numConstraints << " constraints" << std::endl;

	ceres::CostFunction* colorCost = new ceres::AutoDiffCostFunction<ColorRegularizerCost, COLOR_LENGTH, COLOR_LENGTH>(
		new ColorRegularizerCost(COLOR_REGULARIZATION_WEIGHT * numConstraints)
		);
	optimizationDenseProblem.AddResidualBlock(colorCost, NULL, parameters.colorWeights.data());


	ceres::Solver::Summary summary;
	std::cout << "BFMOptimizer >> solving DenseColourTerm . . ." << std::endl;
	ceres::Solve(options, &optimizationDenseProblem, &summary);
	std::cout << summary.BriefReport() << std::endl;
	std::cout << "BFMOptimizer >> DenseColourTerm solved.\n" << std::endl;
}