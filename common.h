#pragma once

#define FALSE	0
#define TRUE	1


/* Dataset Flag */
#define DATASET_FACEGRABBER			0
#define DATASET_KINECT_V2			0
#define DATASET_REALSENSE    		1



/* Display, Saving and Debug Flags */
#define DISPLAY_LANDMARK_ON_FACE    FALSE
#define SAVE_FINAL_REPROJECTION		FALSE
#define DISPLAY_FINAL_REPROJECTION  FALSE
#define SAVE_SCENE_MESH				TRUE
#define SAVE_3D_LANDMARKS_MESH		TRUE

// For OpenCVImgCropper
#define OPENCV_DEBUG_MODE			FALSE
#define DLIB_DEBUG_MODE				FALSE

// Do not wait for input in LandmarkDetector
#define NON_STOP_PROCESS

// in Millimeters
#define COLOURMAP_DISTANCE_MIN      0.0
#define COLOURMAP_DISTANCE_MAX      3 * 0.001


/* Landmark */
#define SHIFT_LANDMARK_BY_1			TRUE
#define REMOVE_OUTLINE_LANDMARKS	TRUE

#define ALL_LANDMARK_COUNT			68
#if (REMOVE_OUTLINE_LANDMARKS)
#define OUTLINE_LANDMARK_COUNT		17
#define LANDMARK_COUNT				51
#else
#define LANDMARK_COUNT				ALL_LANDMARK_COUNT
#endif



/* Optimisation Flags */
#define MAX_NUM_CERES_ITERATIONS	10 // Max num of Ceres iterations

#define DENSE_GEOMETRY_ITERATIONS   1 // how often the dense geometry optimization should be repeated (outside loop)

#define RENDER_SHAPE				TRUE
#define RENDER_EXPRESSION			TRUE
#define RENDER_COLOR				TRUE
#define COLOR_BLENDING_WEIGHT		0.7

// Optimization simplifications (only consider every Nth vertex)
#define OPTIMIZE_DENSE_GEOMETRY_REDUCE_BY   25
#define OPTIMIZE_DENSE_COLOR_REDUCE_BY      25


/* Optimisation Weights */
#define SHAPE_REGULARIZATION_WEIGHT			2.0
#define EXPRESSION_REGULARIZATION_WEIGHT	1.0
#define COLOR_REGULARIZATION_WEIGHT			1.0



/* Basel Face Model */
//#define USE_2017
//#define USE_2009
#define USE_2019

#define MEAN_LENGTH							82971
#define BFM_VERTEX_COUNT					(MEAN_LENGTH / 3)

#define PCA_VARIANCE_LENGTH					199
#define EXPRESSION_PCA_VARIANCE_LENGTH		199

#define PCA_BASIS_LENGTH					(MEAN_LENGTH * 199)

#define EXPRESSION_PCA_BASIS_LENGTH			(MEAN_LENGTH * 100)
#define TRIANGLES							(55040 * 3)

#define SHAPE_LENGTH						199
#define COLOR_LENGTH						199
#define EXPRESSION_LENGTH					100
