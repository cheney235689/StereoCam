#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>

#define cameraIndex 2
#define usingCHECKERBOARD_DIST

#define oriBoardDetect
//#define undistBoardDetect

//int CHECKERBOARD[2]{ 8 - 1,5 - 1 };
//int CHECKERBOARD_DIST = 30;

int CHECKERBOARD[2]{ 7 - 1,4 - 1 };
int CHECKERBOARD_DIST = 55;//mm
int cubeHight = 55 * (-3); 
double t = 0;


//@brief Checks if a matrix is a valid rotation matrix.
//@param R The (3,3) rotation matrix
bool isRotationMatrix(cv::Mat& R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

//@brief Calculates rotation matrix to euler angles.
//The result is the same as MATLAB except the order
//of the euler angles ( x and z are swapped ).
//@param R The input matrix
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

//@brief Get the projected 2D points from 8 vertices of cube
//@param  objp :The 3D points of corners on board
//@param  boardWidth :Refer to the corner numbers of long side
//@param  boradHight :Refer to the corner numbers of short side
//@param  cubeHight :The hight of cube 
//@return The projected 2D points
std::vector<cv::Point3f> getCubePoints(std::vector<cv::Point3f> objp , int boardWidth ,int boradHight , int cubeHight)
{
    std::vector<cv::Point3f> cubePoints;
    cubePoints.push_back(objp[0]);
    cubePoints.push_back(objp[0 + (boardWidth - 1)]);
    cubePoints.push_back(objp[int(boardWidth * boradHight - 1) - (boardWidth - 1)]);
    cubePoints.push_back(objp[int(boardWidth * boradHight - 1)]);
    cubePoints.push_back(objp[0]);
    cubePoints.push_back(objp[0 + (boardWidth - 1)]);
    cubePoints.push_back(objp[int(boardWidth * boradHight - 1) - (boardWidth - 1)]);
    cubePoints.push_back(objp[int(boardWidth * boradHight - 1)]);
    for (int i = 4; i < 8; i++)
    {
        cubePoints[i].z += (float)cubeHight;
    }
    //std::cout << "cube3DPoints:" << cubePoints << std::endl;
    return cubePoints;
}

//@brief Draw cube 
//@param  frame :The frame which we wont to draw
//@param  imagePointProjected :The 2D points of cube vertices
void drawCube(cv::Mat frame , std::vector<cv::Point2f> imagePointProjected)
{
    //conver Point2f to Point
    std::vector<cv::Point> cubicContour;
    for (const auto& point : imagePointProjected) {
        cubicContour.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
    }
    //std::cout << "cubicContour:" << cubicContour << std::endl;

    std::vector<cv::Point> bottomPlane2DPoints;
    bottomPlane2DPoints.push_back(cubicContour[0]);
    bottomPlane2DPoints.push_back(cubicContour[1]);
    bottomPlane2DPoints.push_back(cubicContour[3]);
    bottomPlane2DPoints.push_back(cubicContour[2]);
    //std::cout << "bottomPlane2DPoints:" << bottomPlane2DPoints << std::endl;
    cv::Scalar color(0, 255, 0);
    cv::polylines(frame, bottomPlane2DPoints, true, color, 2);
    std::vector<std::vector<cv::Point>> fillContours = { bottomPlane2DPoints };
    cv::fillPoly(frame, fillContours, color);

    for (int i = 0; i < 4; i++)
    {
        cv::line(frame, cubicContour[i],
            cubicContour[i+4],
            cv::Scalar(255, 0, 0), 6, cv::LINE_8);
    }

    std::vector<cv::Point> TopPlane2DPoints;
    TopPlane2DPoints.push_back(cubicContour[0 + 4]);
    TopPlane2DPoints.push_back(cubicContour[1 + 4]);
    TopPlane2DPoints.push_back(cubicContour[3 + 4]);
    TopPlane2DPoints.push_back(cubicContour[2 + 4]);
    cv::Scalar color2(0, 0, 255);
    cv::polylines(frame, TopPlane2DPoints, true, color2, 2);

}



void rotatePointCloud( std::vector<cv::Point3f> &Points , cv::Point3f centerPoint , cv::Vec3d axis , double angle)
{
    cv::Vec3d normalizedAxis = axis / cv::norm(axis);
    cv::Mat rotationMatrix(3, 3, CV_64F);
    cv::Rodrigues(normalizedAxis * angle, rotationMatrix);
    cv::Mat pointVector(3, 1, CV_64F);

    for (cv::Point3f& point : Points) {
        //Translate first
        point.x -= centerPoint.x;
        point.y -= centerPoint.y;
        point.z -= centerPoint.z;

        pointVector.at<double>(0, 0) = point.x;
        pointVector.at<double>(1, 0) = point.y;
        pointVector.at<double>(2, 0) = point.z;

        cv::Mat rotatedPointVector = rotationMatrix * pointVector;

        point.x = rotatedPointVector.at<double>(0, 0);
        point.y = rotatedPointVector.at<double>(1, 0);
        point.z = rotatedPointVector.at<double>(2, 0);

        //Translate in the end
        point.x += centerPoint.x;
        point.y += centerPoint.y;
        point.z += centerPoint.z;
    }

}




int main() 
{
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1.6;
    int thickness = 1.6;
    int baseline;
    
    float pi = 3.14159265359;
    float conv = 180 / pi;
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;
    std::vector<std::vector<cv::Point3f> > objpointsUndistorted;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;
    std::vector<std::vector<cv::Point2f> > imgpointsUndistorted;

#if cameraIndex == 2
    cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 1078.526938626459, 0, 355.2161141682078,
    0, 1078.601101046102, 674.4569489400669,
    0, 0, 1);
    cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << 0.06614299878168298, 0.05477937325164402, -7.516273243181615e-06, 0.001368218321944865, -0.4476119845729459);
#endif

#if cameraIndex == 0
    cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << 0.04681470056228075, 0.2241082752457147, 0.00021119358731781, 0.002220208231155701, -0.8400049515414876);
    //cv::Mat distortionCoefficients = (cv::Mat1d(1, 14) << -0.1188820694073527, 0.05596077874502294, 0.02384835980608131, -0.001787035705243949, 2.82825454933282, 0.2637579168127516, 0.1571071949882414, 2.923471356008421, 0, 0, 0, 0, 0, 0);
    //cv::Mat distortionCoefficients = (cv::Mat1d(1, 12) << -0.4157772295217677, 0.2720779851692106, -0.01825168572669238, 0.001305527642314787, -0.1131100596074184, 0, 0, 0, -0.002377151705181874, 0.0002960228156220035, 0.1211214164704469, -0.04545074271071811);
    //cv::Mat distortionCoefficients = (cv::Mat1d(1, 14) << -0.3670088825826507, 0.07444780828222007, 0.07755529184994261, -0.0004521688266319207, -0.04483111430383507, 0, 0, 0, 0, 0, 0, 0, -0.1914283805380606, -0.005127584848524948);

    cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 1087.451060203848, 0, 366.4056981691262
        , 0, 1085.083726761653, 593.2663770799095
        , 0, 0, 1);
#endif
    //std::cout<<"distortionCoefficients:"<< distortionCoefficients << std::endl;

    std::vector<cv::Point3f> objp;
#ifndef usingCHECKERBOARD_DIST
        for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
    {
        for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j, i, 0));
    }
#endif // !1



#ifdef usingCHECKERBOARD_DIST
    for (int i{ 0 }; i < CHECKERBOARD[1] * CHECKERBOARD_DIST; i = i + CHECKERBOARD_DIST)
    {
        for (int j{ 0 }; j < CHECKERBOARD[0] * CHECKERBOARD_DIST; j = j + CHECKERBOARD_DIST)
            objp.push_back(cv::Point3f(j, i, 0));

    }
#endif 



    cv::VideoCapture video_capture(cameraIndex);
    video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//640
    video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);//480
    if (!video_capture.isOpened()) {
        std::cout << "failed to capture!" << std::endl;
        return -1;
    }
    cv::Mat UndistortedImg;

    cv::Mat gray , grayUndistorted;
    std::vector<cv::Point2f> corner_pts;
    std::vector<cv::Point2f> corner_ptsUndistorted;
    bool success , successUndistorted;
    while (1)
    {
        cv::Mat frame ;
        video_capture >> frame;
        transpose(frame, frame);
        //cv::flip(frame, frame, 0);
        cv::namedWindow("win W/O drawing", 0);
        cv::resizeWindow("win W/O drawing", cv::Size(720 * 0.6, 1280 * 0.6));
        cv::imshow("win W/O drawing", frame);
        undistort(frame, UndistortedImg, cameraMatrix, distortionCoefficients);
       
        if (frame.empty())
        {
            break;
        }
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

#ifdef oriBoardDetect
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        if (success)
        {

            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            //cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            //std::cout << "objp:" << objp << std::endl;




            imgpoints.push_back(corner_pts);
            cv::Mat rot_mat, tra_mat;
            cv::solvePnP(objp, corner_pts, cameraMatrix, distortionCoefficients, rot_mat, tra_mat, 0, 0);
            //cv::drawFrameAxes(frame, cameraMatrix, distortionCoefficients, rot_mat, tra_mat, 250, 12);
            cv::Vec3f output_v = rotationMatrixToEulerAngles(rot_mat);
            //std::cout << "rot_mat_13:" << rot_mat << std::endl;

            std::vector<cv::Point2f> imagePointProjected;
            std::vector<cv::Point3f> CubePoints = getCubePoints(objp, CHECKERBOARD[0], CHECKERBOARD[1], cubeHight);

            /*rotate the cube*/
            //cv::Vec3d axis = { 0.0 , 0.0 , -1.0 };
            //cv::Point3f centerPoint = { 275 * 0.5 ,  55 , 0 };
            //t = t + (pi / 3000);
            //double angle = sin(t) * conv;
            //std::cout << "angle:" << angle << std::endl;
            //rotatePointCloud(CubePoints, centerPoint, axis, angle);
            /*rotate the cube*/

            cv::projectPoints(CubePoints, rot_mat, tra_mat, cameraMatrix, distortionCoefficients, imagePointProjected);
            //std::cout << "imagePointProjected" << imagePointProjected << std::endl;
            drawCube(frame, imagePointProjected);


            cv::Rodrigues(rot_mat, rot_mat);
            //std::cout << "rot_mat_33:" << rot_mat << std::endl;


            cv::Vec3f EulerV = rotationMatrixToEulerAngles(rot_mat);
            //std::cout<< " degree:" << int(EulerV[1] * conv)<< std::endl;  //Red
            //std::cout << tra_mat<< std::endl;  //Red
            //std::cout << tra_mat.at<double>(0,2) << std::endl;  //Red
            //std::cout << output_v[0] <<"," << output_v[1] << "," << output_v[2] << std::endl;
            //std::cout  << int(output_v[1] * conv) << "," << int(output_v[2] * conv) << std::endl; //W/O blue axis

            //std::cout << int(output_v[0] * conv) << "," << int(output_v[1] * conv) << "," << int(output_v[2] * conv) << std::endl;
            //std::cout << int(output_v[2] * conv) << std::endl;

        // decide text position
            std::string text = std::to_string(int(EulerV[0] * conv)) + "," + std::to_string(int(EulerV[1] * conv)) + "," + std::to_string(int(EulerV[2] * conv));
            std::string text_2 = std::to_string(int(tra_mat.at<double>(0, 2)));
            std::string text_3 = std::to_string(int(tra_mat.at<double>(0, 0))) + "," + std::to_string(int(tra_mat.at<double>(0, 1))) + "," + std::to_string(int(tra_mat.at<double>(0, 2)));
            //Draw text
                //cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
                //cv::Point origin;
                //origin.x = frame.cols / 6 - text_size.width / 2;
                //origin.y = frame.rows / 6 + text_size.height / 2;
                //cv::putText(frame, (text + " degree"), origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);
                ////origin.x = frame.cols / 2 - text_size.width / 2;
                //origin.y = frame.rows / 2 + text_size.height / 2;
                //cv::putText(frame, (text_2 + " mm"), origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);



            std::cout << "-----------" << std::endl;
            std::cout << "rotation(degree):" << text << std::endl;
            std::cout << "translation(mm):" << text_3 << std::endl;
            //std::cout << "Distatance:" << text_2 <<" mm" << std::endl;


        }
        else
        {
            std::cout << "No chessboard detected on original image! " << std::endl;
        }
#endif // oriBoardDetect

#ifdef undistBoardDetect
        cv::cvtColor(UndistortedImg, grayUndistorted, cv::COLOR_BGR2GRAY);
        successUndistorted = cv::findChessboardCorners(grayUndistorted, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_ptsUndistorted, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        if (successUndistorted)
        {
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(grayUndistorted, corner_ptsUndistorted, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            // Displaying the detected corner points on the checker board
            //cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpointsUndistorted.push_back(objp);
            //std::cout << "objp:" << objp << std::endl;




            imgpointsUndistorted.push_back(corner_ptsUndistorted);
            cv::Mat rot_matUndistorted, tra_matUndistorted;
            cv::solvePnP(objp, corner_ptsUndistorted, cameraMatrix, cv::noArray(), rot_matUndistorted, tra_matUndistorted, 0, 0);
            //cv::drawFrameAxes(frame, cameraMatrix, distortionCoefficients, rot_mat, tra_mat, 250, 12);


            std::vector<cv::Point2f> imagePointProjectedUndistorted;
            std::vector<cv::Point3f> CubePoints = getCubePoints(objp, CHECKERBOARD[0], CHECKERBOARD[1], cubeHight);

            /*rotate the cube*/
            //cv::Vec3d axis = { 0.0 , 0.0 , -1.0 };
            //cv::Point3f centerPoint = {275 * 0.5 ,  55 , 0};
            //t = t + (pi / 3000);
            //double angle = sin(t) * conv;
            //std::cout << "angle:" << angle << std::endl;
            //rotatePointCloud(CubePoints, centerPoint, axis, angle);
            /*rotate the cube*/



            cv::projectPoints(CubePoints, rot_matUndistorted, tra_matUndistorted, cameraMatrix, cv::noArray(), imagePointProjectedUndistorted);
            //std::cout << "imagePointProjectedUndistorted" << imagePointProjectedUndistorted << std::endl;
            drawCube(UndistortedImg, imagePointProjectedUndistorted);

        }
        else
        {
            std::cout << "No chessboard detected on undistorted image! " << std::endl;
        }
#endif // undistBoardDetect

        cv::namedWindow("Undistort image", 0);
        cv::resizeWindow("Undistort image", cv::Size(720 * 0.6, 1280 * 0.6));
        cv::imshow("Undistort image", UndistortedImg);

        cv::namedWindow("win", 0);
        cv::resizeWindow("win", cv::Size(720 * 0.6, 1280 * 0.6));
        cv::imshow("win", frame);
        char key = cv::waitKey(30);
    }
	return 0;
}