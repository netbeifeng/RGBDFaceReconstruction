#pragma once

#include <opencv2/opencv.hpp>

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/opencv/cv_image.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <iostream>
#include "OpenCVImgCropper.h"

using namespace dlib;

class LandmarkDetector {
public:
    LandmarkDetector();
    std::vector < std::vector<int>> detect(Tuple img, std::string outputFilename);
    std::vector < std::vector<int>> detectStandalone(std::string filePath, std::string outputFilename);

private:
    std::string dectorPath = "../Model/DLib/shape_predictor_68_face_landmarks.dat";
    frontal_face_detector detector;
    shape_predictor sp;
};