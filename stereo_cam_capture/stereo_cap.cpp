#include <iostream>
#include <string>
#include <fstream>
#include <conio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define cameraIndexL 2
#define cameraIndexR 0



//Image captured path setting
std::string PathL= "D:\\CV_Proj\\stereo_cam_capture\\202401_steroe_setting\\Left\\";
std::string PathR= "D:\\CV_Proj\\stereo_cam_capture\\202401_steroe_setting\\Right\\";
//Capture times
int T = 15;



int main() {
	//1 .Cam streaming
	cv::VideoCapture video_capture_L(cameraIndexL);
	cv::VideoCapture video_capture_R(cameraIndexR);
	video_capture_L.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//640
	video_capture_L.set(cv::CAP_PROP_FRAME_HEIGHT, 720);//480
	video_capture_R.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//640
	video_capture_R.set(cv::CAP_PROP_FRAME_HEIGHT, 720);//480
	if (!video_capture_L.isOpened() || !video_capture_R.isOpened()) {
		std::cout << "failed to capture!" << std::endl;
		return -1;
	}

	//cv::VideoWriter writerL("VideoL.avi", CV_FOURCC('X', '2', '6', '4'), 20.0, cv::Size(1280, 720));
	//cv::VideoWriter writerR("VideoR.avi", CV_FOURCC('M', 'P', '4', '2'), 20.0, cv::Size(1280, 720));
	//cv::VideoWriter videoWriter("output.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 10, cv::Size(1280, 720));





	// 2 .Image Capture
	std::string imgNameL , imgNameR;
	int f = 1;
	int ch;
	while (f < T)
	{
		cv::Mat frame_L;
		video_capture_L >> frame_L;
		transpose(frame_L, frame_L);
		//cv::flip(frame_L, frame_L, 0);

		cv::Mat frame_R;
		video_capture_R >> frame_R;
		transpose(frame_R, frame_R);
		//cv::flip(frame_R, frame_R, 0);

		if (frame_R.empty() || frame_L.empty()) break;
		//cv::imshow("L", frame_L);
		cv::namedWindow("L", 0);
		cv::resizeWindow("L", cv::Size(720 * 0.6, 1280 * 0.6));
		cv::imshow("L", frame_L);
		//cv::imshow("R", frame_R);
		cv::namedWindow("R", 0);
		cv::resizeWindow("R", cv::Size(720 * 0.6, 1280 * 0.6));
		cv::imshow("R", frame_R);
		if (_kbhit())
		{
			ch = _getch();
			if (ch == 65) {  //A
				imgNameL = std::to_string(f);
				imgNameL = PathL + imgNameL;
				cv::imwrite(imgNameL + ".jpg", frame_L);
				imgNameR = std::to_string(f);
				imgNameR = PathR + imgNameR;
				cv::imwrite(imgNameR + ".jpg", frame_R);
				f++;
			}
		}
		char key = cv::waitKey(30);
	}
	//cv::destroyAllWindows();
	//std::cout << "start calibration " << std::endl;


	return 0;
}