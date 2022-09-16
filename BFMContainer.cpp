#include "BFMContainer.h"
#include "common.h"


BFMContainer::BFMContainer() {
    triangles = NULL;
    colorMean = NULL;
    colorPCAVariance = NULL;
    colorPCABasis = NULL;
    shapeMean = NULL;
    shapePCAVariance = NULL;
    shapePCABasis = NULL;
    expressionMean = NULL;
    expressionPCAVariance = NULL;
    expressionPCABasis = NULL;
}

BFMContainer::~BFMContainer() {
    delete[] triangles;
    delete[] colorMean;
    delete[] colorPCAVariance;
    delete[] colorPCABasis;
    delete[] shapeMean;
    delete[] shapePCAVariance;
    delete[] shapePCABasis;
    delete[] expressionMean;
    delete[] expressionPCAVariance;
    delete[] expressionPCABasis;
}

void BFMContainer::loadLandmarks() {
    landmarkVertexIndices = loadIndexList(Landmark_ANLPath);
    optimizationSubsetVertexIndices = loadIndexList(OptimizationSubset_ANLPath);
}

std::vector<int> BFMContainer::loadIndexList(std::string filename) {
    std::ifstream infile(filename);
    std::vector<int> vertexIndices;
    int number;
    while (infile.peek() != EOF)
    {
        if (infile.peek() == '%') {
            // ignore lines that are commented out
            infile.ignore(1000000, '\n');
        }
        else {
            infile >> number;
            vertexIndices.push_back(number);
        }
    }
    infile.close();
    std::cout << "BFMContainer >> " << vertexIndices.size() << " Landmarks loaded." << std::endl;
    return vertexIndices;
}

// LEGACY, not used, because the landmarks strore in H5 file 
// is not within the scope of ibug 300 68 landmark annotaiton
//void BFMContainer::loadLandmarks_BFM2019() {
//    H5::DataSet landmarks = h5_file.openDataSet("metadata/landmarks/json");
//    H5::DataSpace dataSpace = landmarks.getSpace();
//    H5::DataType dataType = landmarks.getDataType();
//    std::cout << "Data name is: " << landmarks.getObjName() << "\n";
//    std::cout << "\n";
//
//    std::string errReport = "";
//    std::string& err = errReport;
//
//    std::string landmarksJSONString = "";
//    landmarks.read(landmarksJSONString, dataType, dataSpace);
//
//    json11::Json landmarksJSON = json11::Json::parse(landmarksJSONString, err);
//    std::cout << "BFMContainer >> " << landmarksJSON.array_items().size() << " Landmarks detected." << std::endl;
//
//    for (json11::Json landmark : landmarksJSON.array_items()) {
//       float x = landmark["coordinates"][0].number_value();
//       float y = landmark["coordinates"][1].number_value();
//       float z = landmark["coordinates"][2].number_value();
//
//       Vector3f coordinate = { x, y, z };
//       landmarkCoordinates.push_back(coordinate);
//       std::cout << "BFMContainer >> Landmark " << landmark["id"].string_value() 
//           << " with (" << x << ", " << y << ", " << z << ") is loaded." << std::endl;
//    }
//    std::cout << "\n";
//}


std::vector<int> BFMContainer::getLandmarkIndices() {
    return landmarkVertexIndices;
}

std::vector<int> BFMContainer::getOptimizationSubsetIndices() {
    return optimizationSubsetVertexIndices;
}

std::vector<Vector3f> BFMContainer::getLandmarkCoordinates(std::vector<Vector3d> vertexList) {
    try {
        std::vector<Vector3f> landmarkCoordinates;
        std::cout << "Vertex List Length -> " << vertexList.size() << "\n";
        for (int landmarkIdx : landmarkVertexIndices) {
            std::cout << "Feteching idx " << landmarkIdx << ". . .\n";
            Vector3d v = vertexList[landmarkIdx];
            std::cout << landmarkIdx << "   " << (float)v.x() << "   " << (float)v.y() << "   " << (float)v.z() << "\n";
            landmarkCoordinates.push_back(Vector3f{ (float)v.x() ,(float)v.y(), (float)v.z() });
        }
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    return landmarkCoordinates;
}

void BFMContainer::loadH5() {
    std::cout << "----- BFMContainer Loading Start -----" << std::endl;
#ifdef USE_2019
    h5_file = H5::H5File(BFM_2019_H5Path, H5F_ACC_RDONLY);
#endif 
#ifdef USE_2015
    h5_file = H5::H5File(BFM_2015_H5Path, H5F_ACC_RDONLY);
#endif 
#ifdef USE_2009
    h5_file = H5::H5File(BFM_2009_H5Path, H5F_ACC_RDONLY);
#endif 
    std::cout << "BFMContainer >> BFM H5 File " << h5_file.getFileName() << "is loaded successfully.\n";
}

void BFMContainer::getTriangleIndices() {
    H5::DataSet dataSet = h5_file.openDataSet("shape/representer/cells");
    std::cout << "BFMContainer >> Load " << dataSet.getObjName() << " successfully with size = " << TRIANGLES << " \n";
    if (triangles == NULL) triangles = new int[TRIANGLES];
    dataSet.read(triangles, H5::PredType::NATIVE_UINT32);
    dataSet.close();
    return ;
}

float* BFMContainer::getDataByPath(std::string path) {
    H5::DataSet dataSet = h5_file.openDataSet(path);
    size_t size = 1;
    auto _npos = std::string::npos;
    if (path.find("mean") != _npos) {
        size = MEAN_LENGTH;
    }
    else if (path.find("pcaVariance") != _npos) {
        if (path.find("expression") != _npos) {
            size = EXPRESSION_PCA_VARIANCE_LENGTH;
        }
        else {
            size = PCA_VARIANCE_LENGTH;
        }
    }
    else if (path.find("pcaBasis") != _npos) {
        if (path.find("expression") != _npos) {
            size = EXPRESSION_PCA_BASIS_LENGTH;
        }
        else {
            size = PCA_BASIS_LENGTH;
        }
    }

    if (size == -1) { throw std::runtime_error("Check your path, no such path is found!"); }

    float* rawData = new float[size];
    std::cout << "BFMContainer >> Load " << dataSet.getObjName() << " successfully with size = " << size << " \n";
    try {
        dataSet.read(rawData, H5::PredType::NATIVE_FLOAT);
    }
     catch (H5::DataSetIException e) {
        dataSet.close();
        e.printErrorStack();
        return nullptr;
    }
    dataSet.close();
    return rawData;
}

double* BFMContainer::convertFloatToDouble(float* data, int size) {
    if (data == nullptr) {
        return nullptr;
    }

    double* double_data = new double[size];
    for (int i = 0; i < size; i++) {
        double_data[i] = (double)data[i];
    }
    delete[] data;

    return double_data;
}

void BFMContainer::loadAllData() {
        shapeMean = convertFloatToDouble((float*)getDataByPath("shape/model/mean"), MEAN_LENGTH);
        shapePCAVariance = convertFloatToDouble((float*)getDataByPath("shape/model/pcaVariance"), PCA_VARIANCE_LENGTH);
        shapePCABasis = convertFloatToDouble((float*)getDataByPath("shape/model/pcaBasis"), PCA_BASIS_LENGTH);

        colorMean = convertFloatToDouble((float*)getDataByPath("color/model/mean"), MEAN_LENGTH);
        colorPCAVariance = convertFloatToDouble((float*)getDataByPath("color/model/pcaVariance"), PCA_VARIANCE_LENGTH);
        colorPCABasis = convertFloatToDouble((float*)getDataByPath("color/model/pcaBasis"), PCA_BASIS_LENGTH);

        expressionMean = convertFloatToDouble((float*)getDataByPath("expression/model/mean"), MEAN_LENGTH);
        expressionPCAVariance = convertFloatToDouble((float*)getDataByPath("expression/model/pcaVariance"), EXPRESSION_PCA_VARIANCE_LENGTH);
        expressionPCABasis = convertFloatToDouble((float*)getDataByPath("expression/model/pcaBasis"), EXPRESSION_PCA_BASIS_LENGTH);

        getTriangleIndices();
        // vertices = new double[MEAN_LENGTH / 3];

        std::cout << "----- BFMContainer Loading End -----" << std::endl;
}

void BFMContainer::dataCheck() {
		std::cout << "BFMContainer >> Check following data, to check whether the data is loaded properly.\n";
		std::cout << "	(1) Shape -> Mean: \n";
		std::cout << "		Yours:   " << shapeMean[0] << " " << shapeMean[1] << " " << shapeMean[2] << "\n";
		std::cout << "		Correct: 6.25138 39.461 105.497\n\n";
		std::cout << "	(2) Shape -> PCA Variance: \n";
		std::cout << "		Yours:   " << shapePCAVariance[0] << "\n";
		std::cout << "		Correct: 295514 \n\n";
		std::cout << "	(3) Shape -> PCA Basis: \n";
		std::cout << "		Yours:   " << shapePCABasis[0] << "\n";
		std::cout << "		Correct: 0.000475607\n\n";
		std::cout << "	(4) Color -> Mean: \n";
		std::cout << "		Yours:   " << colorMean[0] << " " << colorMean[1] << " " << colorMean[2] << std::endl;
		std::cout << "		Correct: 0.849402 0.656286 0.537248\n" << std::endl;
		std::cout << "	(5) Color -> PCA Variance: \n";
		std::cout << "		Yours:   " << colorPCAVariance[0] << "\n";
		std::cout << "		Correct: 175.909 \n\n";
		std::cout << "	(6) Color -> PCA Basis: \n";
		std::cout << "		Yours:   " << colorPCABasis[0] << "\n";
		std::cout << "		Correct: -0.00341921\n\n";
		std::cout << "	(7) Expression -> Mean: \n";
		std::cout << "		Yours:   " << expressionMean[0] << " " << expressionMean[1] << " " << expressionMean[2] << std::endl;
		std::cout << "		Correct: -0.615691 0.175942 0.87737\n" << std::endl;
		std::cout << "	(8) Expression -> PCA Variance: \n";
		std::cout << "		Yours:   " << expressionPCAVariance[0] << "\n";
		std::cout << "		Correct: 101578 \n\n";
		std::cout << "	(9) Expression -> PCA Basis: \n";
		std::cout << "		Yours:   " << expressionPCABasis[0] << "\n";
		std::cout << "		Correct: -0.000262835\n\n";
		std::cout << "	(10) Triangles: \n";
		std::cout << "		Yours:   " << triangles[0] << " " << triangles[1] << "\n";
		std::cout << "		Correct: 5 7351\n\n";
        std::cout << "BFMContainer >> End Of CheckList!" << "\n";
}

BFMParameters BFMContainer::generateRandomParams() {
    BFMParameters params;
    params.shapeWeights = VectorXd::Random(SHAPE_LENGTH) * 0.02;
    params.expressionWeights = VectorXd::Random(EXPRESSION_LENGTH) * 0.02;
    params.colorWeights = VectorXd::Random(COLOR_LENGTH) * 0.02;
    return params;
}


BFMParameters BFMContainer::getZeroParams() {
    BFMParameters params;
    params.shapeWeights = VectorXd::Zero(SHAPE_LENGTH);
    params.expressionWeights = VectorXd::Zero(EXPRESSION_LENGTH);
    params.colorWeights = VectorXd::Zero(COLOR_LENGTH);
    return params;
}


std::vector<Vector3d> BFMContainer::getVertices(BFMParameters params) {
    std::vector<Vector3d> vertices;
    //std::cout << "Getting " << vertexIDs.size() << " vertices from parametrized face" << std::endl;

    MatrixXd shapePCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCAVariance, SHAPE_LENGTH, 1);
    MatrixXd shapePCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCABasis, MEAN_LENGTH, SHAPE_LENGTH);

    MatrixXd expressionPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCAVariance, EXPRESSION_LENGTH, 1);
    MatrixXd expressionPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCABasis, MEAN_LENGTH, EXPRESSION_LENGTH);

    MatrixXd shapeFactors = shapePCAVarMatrix.cwiseSqrt().cwiseProduct(params.shapeWeights);
    MatrixXd expFactors = expressionPCAVarMatrix.cwiseSqrt().cwiseProduct(params.expressionWeights);

    for (int i = 0; i < MEAN_LENGTH / 3; i++) {
        //std::cout << "Loading vertex " << i << std::endl;
        int j = i * 3;
        Vector3d vertex = {
            shapeMean[j] + expressionMean[j] + (shapePCABasisMatrix.row(j) * shapeFactors).value() + (expressionPCABasisMatrix.row(j) * expFactors).value(),
            shapeMean[j + 1] + expressionMean[j + 1] + (shapePCABasisMatrix.row(j + 1) * shapeFactors).value() + (expressionPCABasisMatrix.row(j + 1) * expFactors).value(),
            shapeMean[j + 2] + expressionMean[j + 2] + (shapePCABasisMatrix.row(j + 2) * shapeFactors).value() + (expressionPCABasisMatrix.row(j + 2) * expFactors).value(),
        };
        vertices.push_back(vertex);
    }

    return vertices;
}

int BFMContainer::mapFloatToRange(float f) {
    return floor(f >= 1.0 ? 255 : f * 256.0);
}

double BFMContainer::mapIntToDouble(int i) {
    return i >= 255 ? 1.0 : (i * 1.0 / 256.0);
}

std::vector<Vector3i> BFMContainer::getColors(BFMParameters params) {
    std::vector<Vector3i> colors;
    
    MatrixXd colorPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorPCAVariance, COLOR_LENGTH, 1);
    MatrixXd colorPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorPCABasis, MEAN_LENGTH, COLOR_LENGTH);

    MatrixXd colorFactors = colorPCAVarMatrix.cwiseSqrt().cwiseProduct(params.colorWeights);

    for (int i = 0; i < MEAN_LENGTH / 3; i++) {
        //std::cout << "Loading vertex " << i << std::endl;
        int j = i * 3;
        Vector3i color = {
            mapFloatToRange(colorMean[j] + (colorPCABasisMatrix.row(j) * colorFactors).value()),
            mapFloatToRange(colorMean[j + 1] + (colorPCABasisMatrix.row(j + 1) * colorFactors).value()),
            mapFloatToRange(colorMean[j + 2] + (colorPCABasisMatrix.row(j + 2) * colorFactors).value()),
        };
        colors.push_back(color);
    }

    return colors;
}

std::vector<Vector3d> BFMContainer::getVerticesOfSpecificIds(const std::vector<int> vertexIDs, BFMParameters params){
    std::vector<Vector3d> vertices;
    std::cout << "Getting " << vertexIDs.size() << " vertices from parametrized face" << std::endl;

    MatrixXd shapePCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCAVariance, SHAPE_LENGTH, 1);
    MatrixXd shapePCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCABasis, MEAN_LENGTH, SHAPE_LENGTH);

    MatrixXd expressionPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCAVariance, EXPRESSION_LENGTH, 1);
    MatrixXd expressionPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCABasis, MEAN_LENGTH, EXPRESSION_LENGTH);

    MatrixXd shapeFactors = shapePCAVarMatrix.cwiseSqrt().cwiseProduct(params.shapeWeights);
    MatrixXd expFactors = expressionPCAVarMatrix.cwiseSqrt().cwiseProduct(params.expressionWeights);

    for (int i : vertexIDs) {
        //std::cout << "Loading vertex " << i << std::endl;
        int j = i * 3;
        Vector3d vertex = {
            shapeMean[j] + expressionMean[j] + (shapePCABasisMatrix.row(j) * shapeFactors).value() + (expressionPCABasisMatrix.row(j) * expFactors).value(),
            shapeMean[j + 1] + expressionMean[j + 1] + (shapePCABasisMatrix.row(j+1) * shapeFactors).value() + (expressionPCABasisMatrix.row(j+1) * expFactors).value(),
            shapeMean[j + 2] + expressionMean[j + 2] + (shapePCABasisMatrix.row(j+2) * shapeFactors).value() + (expressionPCABasisMatrix.row(j+2) * expFactors).value(),
        };
        vertices.push_back(vertex);
    }

    return vertices;
}

std::vector<Vector3d> BFMContainer::getVerticesOfMeanFace(const std::vector<int> vertexIDs){
    std::vector<Vector3d> vertices;
    std::cout << "Getting " << vertexIDs.size() << " vertices from mean face" << std::endl;

    for (int i : vertexIDs) {
        //std::cout << "Loading vertex " << i << std::endl;
        int j = i * 3;
        Vector3d vertex = {
            shapeMean[j] + expressionMean[j],
            shapeMean[j + 1] + expressionMean[j + 1],
            shapeMean[j + 2] + expressionMean[j + 2]
        };
        vertices.push_back(vertex);
    }

    return vertices;
}



void BFMContainer::writeToObj(std::string filename, BFMParameters params, Matrix4d pose) {
    std::ofstream objFile(filename + ".obj");

    MatrixXd shapePCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCAVariance, SHAPE_LENGTH, 1);
    MatrixXd shapePCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCABasis, MEAN_LENGTH, SHAPE_LENGTH);

    MatrixXd expressionPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCAVariance, EXPRESSION_LENGTH, 1);
    MatrixXd expressionPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCABasis, MEAN_LENGTH, EXPRESSION_LENGTH);

    MatrixXd shapeResult = shapePCABasisMatrix * (shapePCAVarMatrix.cwiseSqrt().cwiseProduct(params.shapeWeights));
    MatrixXd expResult = expressionPCABasisMatrix * (expressionPCAVarMatrix.cwiseSqrt().cwiseProduct(params.expressionWeights));

    MatrixXd result = shapeResult + expResult;

    MatrixXd expressionMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionMean, MEAN_LENGTH, 1);
    MatrixXd shapeMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapeMean, MEAN_LENGTH, 1);

    MatrixXd vertices = expressionMeanM + shapeMeanM + result;

    MatrixXd colorPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorPCAVariance, COLOR_LENGTH, 1);
    MatrixXd colorPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorPCABasis, MEAN_LENGTH, COLOR_LENGTH);
    MatrixXd colorResult = colorPCABasisMatrix * (colorPCAVarMatrix.cwiseSqrt().cwiseProduct(params.colorWeights));

    MatrixXd colorMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorMean, MEAN_LENGTH, 1);
    MatrixXd colors = colorMeanM + colorResult;


    for (int i = 0; i < MEAN_LENGTH; i += 3) {
        Vector4d vertex = {
            vertices(i),
            vertices(i + 1),
            vertices(i + 2),
            1.0
        };
        vertex = pose * vertex;
        objFile << "v " << vertex[0] << " " 
            << vertex[1] << " " 
            << vertex[2] << " " 
            << colors(i) << " " 
            << colors(i + 1) << " " 
            << colors(i + 2) << "\n";
    }

    for (int i = 0; i < TRIANGLES / 3; i++) {
        objFile << "f " << triangles[i] + 1 << " " 
            << triangles[i + TRIANGLES / 3] + 1 << " "
            << triangles[i + TRIANGLES / 3 * 2] + 1 << "\n";
    }

    objFile.close();
}

std::vector<Vertex> BFMContainer::writeWithBlending(float quotientFromMesh, std::string filename, BFMParameters params, Matrix4d pose, const std::vector<std::pair<Vertex, Vertex>> correspondences) {
    std::ofstream objFile(filename + ".obj");

    MatrixXd shapePCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCAVariance, SHAPE_LENGTH, 1);
    MatrixXd shapePCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCABasis, MEAN_LENGTH, SHAPE_LENGTH);
    MatrixXd shapeMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapeMean, MEAN_LENGTH, 1);

    MatrixXd expressionPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCAVariance, EXPRESSION_LENGTH, 1);
    MatrixXd expressionPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCABasis, MEAN_LENGTH, EXPRESSION_LENGTH);
    MatrixXd expressionMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionMean, MEAN_LENGTH, 1);

    // Render the mean face ( shape + expression )
    MatrixXd vertices = shapeMeanM + expressionMeanM;
#if (RENDER_SHAPE)
    MatrixXd shapeResult = shapePCABasisMatrix * (shapePCAVarMatrix.cwiseSqrt().cwiseProduct(params.shapeWeights));
    vertices += shapeResult;
#endif
#if (RENDER_SHAPE)
    MatrixXd expResult = expressionPCABasisMatrix * (expressionPCAVarMatrix.cwiseSqrt().cwiseProduct(params.expressionWeights));
    vertices += expResult;
#endif

    MatrixXd colorPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorPCAVariance, COLOR_LENGTH, 1);
    MatrixXd colorPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorPCABasis, MEAN_LENGTH, COLOR_LENGTH);
    MatrixXd colorMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorMean, MEAN_LENGTH, 1);
    MatrixXd colors = colorMeanM;
#if (RENDER_COLOR)
    MatrixXd colorResult = colorPCABasisMatrix * (colorPCAVarMatrix.cwiseSqrt().cwiseProduct(params.colorWeights));
    colors += colorResult;
#endif

    std::vector<Vertex> finalVertices;

    for (int i = 0; i < MEAN_LENGTH; i += 3) {
        Vertex meshVertex = correspondences[i / 3].second;

        Vector4d vertex = {
            vertices(i),
            vertices(i + 1),
            vertices(i + 2),
            1.0
        };
        vertex = pose * vertex;

        double R = abs((1 - quotientFromMesh) * colors(i) + quotientFromMesh * mapIntToDouble(meshVertex.color.x()));
        double G = abs((1 - quotientFromMesh) * colors(i + 1) + quotientFromMesh * mapIntToDouble(meshVertex.color.y()));
        double B = abs((1 - quotientFromMesh) * colors(i + 2) + quotientFromMesh * mapIntToDouble(meshVertex.color.z()));

        objFile << "v " << vertex[0] << " "
            << vertex[1] << " "
            << vertex[2] << " "
            << R << " "
            << G << " "
            << B << "\n";

        Vertex finalVertex = Vertex();
        finalVertex.position = Vector4f(vertex[0], vertex[1], vertex[2], 1.0);
        finalVertex.color = Vector4uc(mapFloatToRange(R), mapFloatToRange(G), mapFloatToRange(B), 0.0);
        //std::cout << (1 - quotientFromMesh) * colors(i) << std::endl;
        //std::cout << quotientFromMesh * meshVertex.color.x() << std::endl;
        finalVertices.push_back(finalVertex);
    }

    for (int i = 0; i < TRIANGLES / 3; i++) {
        objFile << "f " << triangles[i] + 1 << " "
            << triangles[i + TRIANGLES / 3] + 1 << " "
            << triangles[i + TRIANGLES / 3 * 2] + 1 << "\n";
    }

    objFile.close();

    return finalVertices;
}


std::vector<Vertex> BFMContainer::writeDistanceColoured(std::string filename, BFMParameters params, Matrix4d pose, const std::vector<std::pair<Vertex, Vertex>> correspondences) {
    std::ofstream objFile(filename + ".obj");

    MatrixXd shapePCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCAVariance, SHAPE_LENGTH, 1);
    MatrixXd shapePCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapePCABasis, MEAN_LENGTH, SHAPE_LENGTH);

    MatrixXd expressionPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCAVariance, EXPRESSION_LENGTH, 1);
    MatrixXd expressionPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionPCABasis, MEAN_LENGTH, EXPRESSION_LENGTH);

    MatrixXd shapeResult = shapePCABasisMatrix * (shapePCAVarMatrix.cwiseSqrt().cwiseProduct(params.shapeWeights));
    MatrixXd expResult = expressionPCABasisMatrix * (expressionPCAVarMatrix.cwiseSqrt().cwiseProduct(params.expressionWeights));

    MatrixXd result = shapeResult + expResult;

    MatrixXd expressionMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(expressionMean, MEAN_LENGTH, 1);
    MatrixXd shapeMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(shapeMean, MEAN_LENGTH, 1);

    MatrixXd vertices = expressionMeanM + shapeMeanM + result;

    MatrixXd colorPCAVarMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorPCAVariance, COLOR_LENGTH, 1);
    MatrixXd colorPCABasisMatrix = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorPCABasis, MEAN_LENGTH, COLOR_LENGTH);
    MatrixXd colorResult = colorPCABasisMatrix * (colorPCAVarMatrix.cwiseSqrt().cwiseProduct(params.colorWeights));

    MatrixXd colorMeanM = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(colorMean, MEAN_LENGTH, 1);
    MatrixXd colors = colorMeanM + colorResult;

    std::vector<Vertex> finalVertices;

    for (int i = 0; i < MEAN_LENGTH; i += 3) {
        Vertex meshVertex = correspondences[i / 3].second;

        Vector4d vertex = {
            vertices(i),
            vertices(i + 1),
            vertices(i + 2),
            1.0
        };
        vertex = pose * vertex;

        double distance = (vertex - meshVertex.position.cast<double>()).norm();
        double R, G, B;
        distanceToColour(distance, R, G, B);

        objFile << "v " << vertex[0] << " "
            << vertex[1] << " "
            << vertex[2] << " "
            << R << " "
            << G << " "
            << B << "\n";

        Vertex finalVertex = Vertex();
        finalVertex.position = Vector4f(vertex[0], vertex[1], vertex[2], 1.0);
        finalVertex.color = Vector4uc(mapFloatToRange(R), mapFloatToRange(G), mapFloatToRange(B), 0.0);
        //std::cout << (1 - quotientFromMesh) * colors(i) << std::endl;
        //std::cout << quotientFromMesh * meshVertex.color.x() << std::endl;
        finalVertices.push_back(finalVertex);
    }

    for (int i = 0; i < TRIANGLES / 3; i++) {
        objFile << "f " << triangles[i] + 1 << " "
            << triangles[i + TRIANGLES / 3] + 1 << " "
            << triangles[i + TRIANGLES / 3 * 2] + 1 << "\n";
    }

    objFile.close();

    return finalVertices;
}

/**
 * @brief Maps a distance to a rainbow colour map (blue - green - red)
 * calibrated for real-world sizes in meter scale
 * Adapted from https://www.particleincell.com/2014/colormap/
 */
void BFMContainer::distanceToColour(double distance, double& R, double& G, double& B){
    double a = (1.0-
        max(min(
            (distance - COLOURMAP_DISTANCE_MIN) / (COLOURMAP_DISTANCE_MAX - COLOURMAP_DISTANCE_MIN)
        , 1.0), 0.0)) * 4.0;	// map to value range, invert and group
    int X = max((int)floor(a), 0); // gradient group
    double Y = a-X; // location within the gradient
    switch(X)
    {
        case 0: R=1.0;G=Y;B=0;break;
        case 1: R=1.0-Y;G=1.0;B=0;break;
        case 2: R=0;G=1.0;B=Y;break;
        case 3: R=0;G=1.0-Y;B=1.0;break;
        default: R=0;G=0;B=1.0;break;
    }
    return;
}