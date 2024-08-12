#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
int CHECKERBOARD[2]{ 4 - 1 , 5 - 1 };
int CHECKERBOARD_DIST = 20;

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat& R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);

}




int main()
{
    //for text
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1.6;
    int thickness = 1.6;
    int baseline;
    //for conv.
    float pi = 3.14159265359;
    float conv = 180 / pi;
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;
    cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << -0.4179366321635395, 0.4372769653094547
        , 0.0009091493761305932, -0.001015177757627367
        , -0.6586136742021954);
    cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 584.4973399220805, 0, 302.5346832863321
        , 0, 584.0933270913546, 260.4282931260725
        , 0, 0, 1);
    std::vector<cv::Point3f> objp;
    //for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
    //{
    //    for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
    //        objp.push_back(cv::Point3f(j, i, 0));
    //}

    for (int i{ 0 }; i < CHECKERBOARD[1] * CHECKERBOARD_DIST; i = i + CHECKERBOARD_DIST)
    {
        for (int j{ 0 }; j < CHECKERBOARD[0] * CHECKERBOARD_DIST; j = j + CHECKERBOARD_DIST)
            objp.push_back(cv::Point3f(j, i, 0));

    }
    cv::VideoCapture video_capture(0);
    video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);//640
    video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);//480
    if (!video_capture.isOpened()) {
        std::cout << "failed to capture!" << std::endl;
        return -1;
    }

    cv::Mat gray;
    std::vector<cv::Point2f> corner_pts;
    bool success;

    //for video writer
    cv::VideoWriter writer("VideoTest_2.avi", CV_FOURCC('M', 'P', '4', '2'), 20.0, cv::Size(640, 480));
    //for image padding

    int top, bottom, left, right;
    top = (int)(480*0.6);
    bottom = (int)(480 * 0.6);
    left = (int)(640*0.6);
    right = (int)(640 * 0.6);
    cv::RNG rng(12345);
    cv::Scalar value(rng.uniform(254, 255), rng.uniform(254, 255), rng.uniform(254, 255));

    while (1)
    {


        cv::Mat frame;
        video_capture >> frame;
        if (frame.empty()) break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        //for padding
        cv::Mat fram_padded = frame;
        cv::copyMakeBorder(fram_padded, fram_padded, top, bottom, left, right, cv::BORDER_CONSTANT, value);
        cv::resize(fram_padded, fram_padded, cv::Size(640, 480));
        writer << fram_padded;
        //



        
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        cv::imshow("win W/O drawing", frame);
        if (success)
        {
            

            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            //cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
            cv::Mat rot_mat, tra_mat;
            cv::solvePnP(objp, corner_pts, cameraMatrix, distortionCoefficients, rot_mat, tra_mat, 0, 0);
            cv::drawFrameAxes(frame, cameraMatrix, distortionCoefficients, rot_mat, tra_mat, 50, 3);
            cv::Vec3f output_v = rotationMatrixToEulerAngles(rot_mat);


            cv::Rodrigues(rot_mat, rot_mat);
            cv::Vec3f EulerV = rotationMatrixToEulerAngles(rot_mat);
            //std::cout<< " degree:" << int(EulerV[1] * conv)<< std::endl;  //Red
            //std::cout << tra_mat<< std::endl;  //Red
            std::cout << tra_mat.at<double>(0, 2) << std::endl;  //Red
            //std::cout << output_v[0] <<"," << output_v[1] << "," << output_v[2] << std::endl;
            //std::cout  << int(output_v[1] * conv) << "," << int(output_v[2] * conv) << std::endl; //W/O blue axis

            //std::cout << int(output_v[0] * conv) << "," << int(output_v[1] * conv) << "," << int(output_v[2] * conv) << std::endl;
            //std::cout << int(output_v[2] * conv) << std::endl;
            
        // decide text position
            std::string text = std::to_string(int(EulerV[1] * conv));
            std::string text_2 = std::to_string(int(tra_mat.at<double>(0, 2)));
            cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
            cv::Point origin;
            origin.x = frame.cols / 6 - text_size.width / 2;
            origin.y = frame.rows / 6 + text_size.height / 2;
            cv::putText(frame, (text + " degree"), origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);
            //origin.x = frame.cols / 2 - text_size.width / 2;
            origin.y = frame.rows / 2 + text_size.height / 2;
            cv::putText(frame, (text_2 + " mm"), origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);
       
        }
        else
        {
            std::cout << "No chessboard detected! " << std::endl;
        }





        cv::imshow("win", frame);
        char key = cv::waitKey(30);
    }
    //while (1)
    //{
    //    cv::Mat frame;
    //    video_capture >> frame;
    //    if (frame.empty()) break;
    //    std::vector<int> markerIds;
    //    // Aruco marker corners vector
    //    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    //    std::vector<std::vector<cv::Point3f>> The_3d_points;
    //    // Aruco detect markers
    //    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    //    cv::imshow("win", frame);
    //     char key = cv::waitKey(30);
    //}


    return 0;
}