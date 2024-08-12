#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define UsingChessboardDist

using namespace cv;
using namespace std;

int main()
{
	//int CHECKERBOARD[2]{ 8 - 1,5 - 1 };
	//int CHECKERBOARD_DIST = 30;//mm

	int CHECKERBOARD[2]{ 7 - 1,4 - 1 };
	int CHECKERBOARD_DIST = 55;//mm


	//Load 3D object point
	vector< vector< Point3f > >objpoints_L , objpoints_R  ;
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


	//Load image point
	vector< vector< Point2f > > L_imagePoints, R_imagePoints;
	vector< Point2f > L_corners, R_corners;
	vector< vector< Point2f > > L_img_points, R_img_points;
	Mat K1 , K2 ,R, F, E;
	cv::Mat cameraMatrix_Left = cv::Mat::zeros(3, 3, CV_64F);
	cameraMatrix_Left.at<double>(0, 0) = 1086.730502642941; //fx
	cameraMatrix_Left.at<double>(0, 1) = 0; //s
	cameraMatrix_Left.at<double>(0, 2) = 366.4758216167897; //cx
	cameraMatrix_Left.at<double>(1, 1) = 1086.069457882601; //fy
	cameraMatrix_Left.at<double>(1, 2) = 590.1080630581687;//cy
	cameraMatrix_Left.at<double>(2, 2) = 1;

	cv::Mat cameraMatrix_Right = cv::Mat::zeros(3, 3, CV_64F);
	cameraMatrix_Right.at<double>(0, 0) = 1087.451060203848; //fx
	cameraMatrix_Right.at<double>(0, 1) = 0; //s
	cameraMatrix_Right.at<double>(0, 2) = 366.4056981691262; //cx
	cameraMatrix_Right.at<double>(1, 1) = 1085.083726761653; //fy
	cameraMatrix_Right.at<double>(1, 2) = 593.2663770799095;//cy
	cameraMatrix_Right.at<double>(2, 2) = 1;


	Vec3d T;
	Mat D1, D2;
	cv::Mat distortionCoefficients_Left = cv::Mat::zeros(5, 1, CV_64F);
	distortionCoefficients_Left.at<double>(0, 0) = 0.05332707310190592;
	distortionCoefficients_Left.at<double>(1, 0) = 0.2464045999527728;
	distortionCoefficients_Left.at<double>(2, 0) = -0.003004407356558586;
	distortionCoefficients_Left.at<double>(3, 0) = 0.004253737368511064;
	distortionCoefficients_Left.at<double>(4, 0) = -1.105932171586019;

	cv::Mat distortionCoefficients_Right = cv::Mat::zeros(5, 1, CV_64F);
	distortionCoefficients_Right.at<double>(0, 0) = 0.04681470056228075;
	distortionCoefficients_Right.at<double>(1, 0) = 0.2241082752457147;
	distortionCoefficients_Right.at<double>(2, 0) = 0.00021119358731781;
	distortionCoefficients_Right.at<double>(3, 0) = 0.002220208231155701;
	distortionCoefficients_Right.at<double>(4, 0) = -0.8400049515414876;
	bool L_success , R_success;

	//std::string L_Imgpath = "C:\\Users\\sis10173\\projects\\stereo_cam_capture\\Left_cap\\";
	//std::string R_Imgpath = "C:\\Users\\sis10173\\projects\\stereo_cam_capture\\Right_cap\\";
	
	//std::string L_Imgpath = "D:\\CV_Proj\\stereo_cam_capture\\1121_stereo_cap\\Left\\";
	//std::string R_Imgpath = "D:\\CV_Proj\\stereo_cam_capture\\1121_stereo_cap\\Right\\";
	


	std::string L_Imgpath = "D:\\CV_Proj\\stereo_cam_calibration\\202401_steroe_setting\\Left\\";
	std::string R_Imgpath = "D:\\CV_Proj\\stereo_cam_calibration\\202401_steroe_setting\\Right\\";

	//std::string L_Imgpath = "D:\\CV_Proj\\stereo_cam_capture\\1211_stereo\\Left\\inputFrame";
	//std::string R_Imgpath = "D:\\CV_Proj\\stereo_cam_capture\\1211_stereo\\Right\\inputFrame";


	//Load left image points
	std::vector<cv::String> L_images;
	std::string L_file = L_Imgpath + "*.jpg";
	cv::glob(L_file, L_images);
	cv::Mat L_frame, L_gray;

	int L_results_num;
	for (int i{ 0 }; i < L_images.size(); i++)
	{
		L_frame = cv::imread(L_images[i]);
		cv::cvtColor(L_frame, L_gray, cv::COLOR_BGR2GRAY);
		L_success = cv::findChessboardCorners(L_gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]),
		L_corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (L_success)
		{
			cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

			// refining pixel coordinates for given 2d points.
			cv::cornerSubPix(L_gray, L_corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

			// Displaying the detected corner points on the checker board
			cv::drawChessboardCorners(L_frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), L_corners, L_success);

			objpoints_L.push_back(objp);
			L_img_points.push_back(L_corners);
		}
		else
		{
			std::cout << "Left cam:loading img point not success " << std::endl;
		}
		//Result output
		//std::string L_result_imgName;
		//L_result_imgName = std::to_string(L_results_num++);
		//cv::imwrite(ResultPath + L_results_num + ".jpg", L_frame);
		//cv::waitKey(15);
	}

	//Load right image points
	std::vector<cv::String> R_images;
	std::string R_file = R_Imgpath + "*.jpg";
	cv::glob(R_file, R_images);
	cv::Mat R_frame, R_gray;
	
	int R_results_num;
	for (int i{ 0 }; i < R_images.size(); i++)
	{
		R_frame = cv::imread(R_images[i]);
		cv::cvtColor(R_frame, R_gray, cv::COLOR_BGR2GRAY);
		R_success = cv::findChessboardCorners(R_gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]),
			R_corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (R_success)
		{
			cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

			// refining pixel coordinates for given 2d points.
			cv::cornerSubPix(R_gray, R_corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

			// Displaying the detected corner points on the checker board
			cv::drawChessboardCorners(R_frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), R_corners, R_success);

			objpoints_R.push_back(objp);
			R_img_points.push_back(R_corners);
		}
		else
		{
			std::cout << "Right cam:loading img point not success " << std::endl;
		}
		//Result output
		//std::string R_result_imgName;
		//R_result_imgName = std::to_string(R_results_num++);
		//cv::imwrite(ResultPath + R_results_num + ".jpg", R_frame);
		//cv::waitKey(15);
	}




	//cv::stereoCalibrate(objpoints_R, L_img_points, R_img_points,
	//				K1, D1, K2, D2, L_frame.size(), R, T, E, F, CV_CALIB_FIX_FOCAL_LENGTH | CV_CALIB_FIX_PRINCIPAL_POINT,
	//					cv::TermCriteria((cv::TermCriteria::COUNT + cv::TermCriteria::EPS), 30, 9.999999999999e-7));
	//cv::stereoCalibrate(objpoints_R, L_img_points, R_img_points,
	//	K1, D1, K2, D2, L_frame.size(), R, T, E, F, CV_CALIB_USE_INTRINSIC_GUESS,
	//	cv::TermCriteria((cv::TermCriteria::COUNT + cv::TermCriteria::EPS), 30, 9.999999999999e-7));
	cv::stereoCalibrate(objpoints_R, L_img_points, R_img_points,
		cameraMatrix_Left, distortionCoefficients_Left, cameraMatrix_Right, distortionCoefficients_Right, L_frame.size(), R, T, E, F);


	std::ofstream ofs;
	ofs.open("stereo_calib_202401_setting2.txt");
	if (!ofs.is_open()) {
		std::cout << "Failed to open file.\n";
	}
	else {
		ofs << "Rotation vector : " << R << "\n";
		ofs << "Translation vector : " << T << "\n";
		ofs << "K1: " << K1 << "\n";
		ofs << "K2 : " << K2 << "\n";
		ofs << "D1 : " << D1 << "\n";
		ofs << "D2 : " << D2 << "\n";
		ofs.close();
	}



	return 0;
}