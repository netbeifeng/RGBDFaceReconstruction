#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include "LandmarkDetector.h"
#include "common.h"

LandmarkDetector::LandmarkDetector() {
    try {
        detector = get_frontal_face_detector();
        deserialize(dectorPath) >> sp;
    } catch (std::exception& e) {
        std::cout << "\nNo Detector Avaliable!" << std::endl;
        std::cout << e.what() << std::endl;
    }
}

std::vector < std::vector<int>> LandmarkDetector::detect(Tuple img, std::string outputFilename) {
    //string filenameIn = "../Data/Initial/000_00_image.png";
    cv::Mat cvImg = img.face;
    Coordinate c = img.s_point;
    std::vector<std::vector<int>> landmarks;
    try {
        image_window win;
        
        std::cout << "----- Dlib Face Landmark Detector Start -----" << std::endl;

        std::cout << "DLib >> processing image " << std::endl;
        array2d<rgb_pixel> img;
        assign_image(img, dlib::cv_image<bgr_pixel>(cvImg));
        //pyramid_up(img);

        std::vector<rectangle> dets = detector(img);
        std::cout << "DLib >> Number of faces detected: " << dets.size() << std::endl;

        // Now we will go ask the shape_predictor to tell us the pose of
        // each face we detected.
        std::vector<full_object_detection> shapes;
        for (unsigned long j = 0; j < dets.size(); ++j)
        {
            full_object_detection shape = sp(img, dets[j]);
            std::cout << "DLib >> Number of landmarks: " << shape.num_parts() << std::endl;
            std::cout << "DLib >> Pixel position of 1. landmark:  " << shape.part(0) << std::endl;
            std::cout << "DLib >> Pixel position of 2. landmark: " << shape.part(1) << std::endl;
            std::cout << "DLib >> Pixel position of 28. landmark: " << shape.part(27) << std::endl;
            std::cout << "DLib >> Pixel position of 29. landmark: " << shape.part(28) << std::endl;
            std::cout << "DLib >> Pixel position of 30. landmark: " << shape.part(29) << std::endl;
            std::cout << "DLib >> Pixel position of 31. landmark: " << shape.part(30) << std::endl;
            std::cout << "DLib >> Pixel position of 40. landmark: " << shape.part(39) << std::endl;
            std::cout << "DLib >> Pixel position of 43. landmark: " << shape.part(42) << std::endl;
            std::cout << "DLib >> Pixel position of 68. landmark: " << shape.part(67) << std::endl;

            std::string filename = outputFilename + ".pts";

            std::ofstream ofs(filename);

            if (!ofs.is_open()) {
                throw std::runtime_error("Output Stream not open");
            }

            ofs << "version 1\nn_points:  68\n{" << std::endl;

            for (unsigned int i = 0; i < shape.num_parts(); i++) {
                std::vector<int> pt;
                point p = shape.part(i);
#if (SHIFT_LANDMARK_BY_1)
                int x = p.x() + c.x + 1;
                int y = p.y() + c.y - 1;
#else
                int x = p.x() + c.x;
                int y = p.y() + c.y;
#endif
                ofs << x << " " << y << std::endl;
                pt.push_back(x);
                pt.push_back(y);
                landmarks.push_back(pt);
            }

            ofs << "}" << std::endl;
            ofs.close();
            std::cout << "Dlib DataWriter >> ### Write landmarks to " << filename << " ###" << std::endl;

            // You get the idea, you can get all the face part locations if
            // you want them.  Here we just store them in shapes so we can
            // put them on the screen.
            shapes.push_back(shape);
        }
        std::cout << "----- Dlib Face Landmark Detector End -----" << std::endl;
        
#if (DLIB_DEBUG_MODE)
        // Now let's view our face poses on the screen.
        win.clear_overlay();
        win.set_image(img);
        win.add_overlay(render_face_detections(shapes));
#ifndef NON_STOP_PROCESS
        cout << "Dlib >> Hit enter to process the next frame..." << endl;
        cin.get();
#endif
#endif
    }
    catch (std::exception& e) {
        std::cout << "\nSomething was wrong!" << std::endl;
        std::cout << e.what() << std::endl;
    }

    return landmarks;
}

std::vector < std::vector<int>> LandmarkDetector::detectStandalone(std::string filenameIn, std::string outputFilename) {
    std::vector<std::vector<int>> landmarks;
    try {
        image_window win;

        std::cout << "----- Dlib Face Landmark Detector Start -----" << std::endl;

        std::cout << "DLib >> processing image " << std::endl;
        array2d<rgb_pixel> img;
        load_image(img, filenameIn);
        //assign_image(img, dlib::cv_image<bgr_pixel>(cvImg));
        //pyramid_up(img);

        std::vector<rectangle> dets = detector(img);
        std::cout << "DLib >> Number of faces detected: " << dets.size() << std::endl;

        // Now we will go ask the shape_predictor to tell us the pose of
        // each face we detected.
        std::vector<full_object_detection> shapes;
        for (unsigned long j = 0; j < dets.size(); ++j)
        {
            full_object_detection shape = sp(img, dets[j]);
            std::cout << "DLib >> Number of landmarks: " << shape.num_parts() << std::endl;
            std::cout << "DLib >> Pixel position of 1. landmark:  " << shape.part(0) << std::endl;
            std::cout << "DLib >> Pixel position of 2. landmark: " << shape.part(1) << std::endl;
            std::cout << "DLib >> Pixel position of 28. landmark: " << shape.part(27) << std::endl;
            std::cout << "DLib >> Pixel position of 29. landmark: " << shape.part(28) << std::endl;
            std::cout << "DLib >> Pixel position of 30. landmark: " << shape.part(29) << std::endl;
            std::cout << "DLib >> Pixel position of 31. landmark: " << shape.part(30) << std::endl;
            std::cout << "DLib >> Pixel position of 40. landmark: " << shape.part(39) << std::endl;
            std::cout << "DLib >> Pixel position of 43. landmark: " << shape.part(42) << std::endl;
            std::cout << "DLib >> Pixel position of 68. landmark: " << shape.part(67) << std::endl;

            std::string filename = outputFilename + ".pts";

            std::ofstream ofs(filename);

            if (!ofs.is_open()) {
                throw std::runtime_error("Output Stream not open");
            }

            ofs << "version 1\nn_points:  68\n{" << std::endl;

            for (unsigned int i = 0; i < shape.num_parts(); i++) {
                std::vector<int> pt;
                point p = shape.part(i);
                int x = p.x();
                int y = p.y();
                ofs << x << " " << y << std::endl;
            }

            ofs << "}" << std::endl;
            ofs.close();
            std::cout << "Dlib DataWriter >> ### Write landmarks to " << filename << " ###" << std::endl;

            // You get the idea, you can get all the face part locations if
            // you want them.  Here we just store them in shapes so we can
            // put them on the screen.
            shapes.push_back(shape);
        }
        std::cout << "----- Dlib Face Landmark Detector End -----" << std::endl;

#if (DLIB_DEBUG_MODE)
        // Now let's view our face poses on the screen.
        win.clear_overlay();
        win.set_image(img);
        win.add_overlay(render_face_detections(shapes));
#ifndef NON_STOP_PROCESS
        cout << "Dlib >> Hit enter to process the next frame..." << endl;
        cin.get();
#endif
#endif
    }
    catch (std::exception& e) {
        std::cout << "\nSomething was wrong!" << std::endl;
        std::cout << e.what() << std::endl;
    }

    return landmarks;
}