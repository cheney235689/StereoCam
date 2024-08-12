#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#define UsingChessboardDist

// Defining the dimensions of checkerboard
//int CHECKERBOARD[2]{ 11-1,8-1 };
//int CHECKERBOARD[2]{ 8 - 1,5 - 1 };
int CHECKERBOARD[2]{ 7 - 1,4 - 1 };
int CHECKERBOARD_DIST = 55;//mm


std::string Imgpath = "D:\\CV_Proj\\Calib\\1212_S2\\Left\\inputFrame\\";
std::string ResultPath = "D:\\CV_Proj\\Calib\\1212_S2\\Left\\outputFrame\\";
std::string OutputParamPath = "D:\\CV_Proj\\Calib\\1212_S2\\Left\\Param.txt";

int main()
{

    std::cout << __cplusplus << std::endl;

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;

#ifndef UsingChessboardDist
    for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
    {
        for (int j{ 0 }; j < CHECKERBOARD[0]; j++)       
            objp.push_back(cv::Point3f(j, i, 0));
            
    }
#else
    for (int i{ 0 }; i < CHECKERBOARD[1] * CHECKERBOARD_DIST; i = i + CHECKERBOARD_DIST)
    {
        for (int j{ 0 }; j < CHECKERBOARD[0] * CHECKERBOARD_DIST; j = j + CHECKERBOARD_DIST)
            objp.push_back(cv::Point3f(j, i, 0));

    }

#endif // !1

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    std::string path = Imgpath + "*.jpg";

    cv::glob(path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;
    bool success;
    int k = 0;
    // Looping over all the images in the directory
    for (int i{ 0 }; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checker board
        */
        if (success)
        {
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
        else
        {
            std::cout << "not success " << std::endl;
        }

        cv::imshow("Image", frame);
        std::string imgName;
        imgName = std::to_string(k++);
        cv::imwrite(ResultPath + imgName + ".jpg", frame);
        cv::waitKey(15);
    }

    cv::destroyAllWindows();
    cv::Mat cameraMatrix, distCoeffs, R, T;

    /*
     * Performing camera calibration by
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the
     * detected corners (imgpoints)
    */
    double RPE;
    RPE = cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T  );
    std::cout << "RPE : " << RPE << std::endl;
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    std::cout << "Rotation vector : " << R << std::endl;
    std::cout << "Translation vector : " << T << std::endl;

    std::ofstream ofs;
    //ofs.open("1030_ yellowCam.txt");
    ofs.open(OutputParamPath);
    if (!ofs.is_open()) {
        std::cout << "Failed to open file.\n";
    }
    else {
        ofs << "cameraMatrix : " << cameraMatrix << "\n";
        ofs << "distCoeffs : " << distCoeffs << "\n";
        ofs << "Rotation vector : " << R << "\n";
        ofs << "Translation vector : " << T << "\n";
        ofs.close();
    }

    return 0;
}