#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <conio.h>

#define cameraIndex 1
#define calibration


// Defining the dimensions of checkerboard
//int CHECKERBOARD[2]{ 11-1,8-1 };
//int CHECKERBOARD[2]{ 8 - 1,5 - 1 };

int CHECKERBOARD[2]{ 7 - 1,4 - 1 };
int CHECKERBOARD_DIST = 55;//mm
//std::string Imgpath = "D:\\CV_Proj\\FisheyeCalib\\1030_ yellowCam\\inputFrame\\";
//std::string ResultPath = "D:\\CV_Proj\\FisheyeCalib\\1030_ yellowCam\\outputFrame\\";
//std::string OutputParamPath = "D:\\CV_Proj\\FisheyeCalib\\1030_ yellowCam\\Param.txt";

//std::string Imgpath = "D:\\CV_Proj\\FisheyeCalib\\Left_cap_1110\\inputFrame\\";
//std::string ResultPath = "D:\\CV_Proj\\FisheyeCalib\\Left_cap_1110\\outputFrame\\";
//std::string OutputParamPath = "D:\\CV_Proj\\FisheyeCalib\\Left_cap_1110\\Param.txt";


std::string Imgpath = "D:\\CV_Proj\\FisheyeCalib\\1201CAP\\inputFrame\\";
std::string ResultPath = "D:\\CV_Proj\\FisheyeCalib\\1201CAP\\outputFrame\\";
std::string OutputParamPath = "D:\\CV_Proj\\FisheyeCalib\\1201CAP\\Param.txt";


int main()
{
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
        cv::waitKey(16);
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
    cv::Size corrected_size(720 , 1280 );  //720 , 1280

    cv::Mat mapx, mapy;
    cv::Mat corrected;

    int flag = 0;
    flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    //flag |= cv::fisheye::CALIB_CHECK_COND;
    flag |= cv::fisheye::CALIB_FIX_SKEW;
    //flag |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
   
   
    //RPE = cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
    RPE = cv::fisheye::calibrate(objpoints, imgpoints, cv::Size(gray.cols, gray.rows), cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), flag, cv::TermCriteria(3, 20, 1e-6));
    
    double balance = 0.0;   // range between the min focal length and the max focal length
    double fov_scale = 0.0;  // Divisor for new focal length
    cv::Size new_corrected_size(720   , 1280  );
    cv::Mat P;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, corrected_size , cv::Matx33d::eye() , P , balance , new_corrected_size, fov_scale );
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Matx33d::eye(), P, new_corrected_size, CV_16SC2, mapx, mapy);

    //cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Matx33d::eye(), cameraMatrix, corrected_size, CV_16SC2, mapx, mapy);
    std::cout << "cv::Size(gray.rows, gray.cols) : " << cv::Size(gray.rows, gray.cols) << std::endl;
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



    cv::VideoCapture video_capture(cameraIndex);
    video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//640
    video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);//480

    int ch;
    int imgIndex = 0;
    std::string imgName;
    
    while (1)
    {
        cv::Mat frame;
        video_capture >> frame;
        transpose(frame, frame);
        cv::flip(frame, frame, 0);
        if (!video_capture.isOpened()) {
            std::cout << "failed to capture!" << std::endl;
            return -1;
        }
        cv::Mat UndistortedImg;

     
        cv::remap(frame, corrected, mapx, mapy, cv::INTER_LINEAR , cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        //cv::fisheye::undistortImage(frame , corrected , cameraMatrix, distCoeffs);

        //cv::Mat enlargedImage = { 1280 * 2, 720 * 2, corrected.type(), cv::Scalar(0) };
        //cv::Mat ROI = enlargedImage(cv::Rect( 360 ,640  ,1080 , 1920  ));
        //cv::Mat mesk; 
        //cv::cvtColor(corrected, mesk, cv::COLOR_BGR2GRAY);
        //corrected.copyTo(ROI, mesk);
        //cv::namedWindow("enlargedImage", 0);
        ////cv::resizeWindow("enlargedImage", cv::Size(720 * 0.6, 1280 * 0.6));
        //cv::imshow("enlargedImage", enlargedImage);

        if (_kbhit())
        {
            ch = _getch();
            if (ch == 65) {
                imgName = std::to_string(imgIndex++);
                cv::imwrite(imgName + "_dist.jpg", corrected);
                cv::imwrite(imgName + "_ori.jpg", frame);
                //cv::imwrite(imgName + "enlargedImage.jpg", enlargedImage);
            }
        }

        std::cout << "corrected.size : " << corrected.size << std::endl;


        cv::namedWindow("win", 0);
        cv::resizeWindow("win", cv::Size(720 * 0.6, 1280 * 0.6));
        cv::imshow("win", frame);

        cv::namedWindow("Undistort image", 0);
        //cv::resizeWindow("Undistort image", cv::Size(1280 * 0.6, 720 * 0.6));
        cv::imshow("Undistort image", corrected);
        char key = cv::waitKey(30);
    }
    
    return 0;
}