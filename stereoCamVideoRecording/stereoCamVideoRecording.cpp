#include <iostream>
#include <string>
#include <fstream>
#include <conio.h>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define cameraIndexL 2
#define cameraIndexR 0



//Image captured path setting
std::string PathL = "D:\\CV_Proj\\stereoCamVideoRecording\\";
std::string PathR = "D:\\CV_Proj\\stereoCamVideoRecording\\";
//Capture times
int T = 25;
int stopFlag = 0;
void captureAndDisplay(int cameraIndex , cv::Mat &currentFrame   )
{
	int frameCount = 0;
	cv::VideoCapture cap(cameraIndex);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//640
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);//480

	if (!cap.isOpened()) {
		std::cerr << "Can't open camera" << cameraIndex << "." << std::endl;
		return;
	}
	std::string windowName = "Camera_";
	windowName = windowName + std::to_string(cameraIndex);
	cv::namedWindow(windowName, cv::WINDOW_NORMAL);

	std::string mp4Name = windowName + ".mp4";
	cv::VideoWriter videoWriter(mp4Name, cv::VideoWriter::fourcc('M', 'P', '4', '2'), 30,
		cv::Size(720, 1280));

	

	while (true) {
		cv::Mat frame;
		cap >> frame;
		transpose(frame, frame);
		videoWriter << frame;
		currentFrame = frame;

		if (frame.empty()) {
			std::cerr << "can't get frame from camera" << cameraIndex << std::endl;
			break;
		}

		cv::imshow(windowName, frame);
		frameCount++;
		cv::waitKey(1);
		//if (cv::waitKey(1) == 27) {
		//	break;
		//}
		if (stopFlag == 1) {
			std::cout <<"frame number of " << mp4Name << ":" << frameCount << std::endl;
			break;
		}
	}

	cap.release();
	cv::destroyWindow(windowName);
}






int main() {
	cv::Mat frameL, frameR;
	std::thread thread1(captureAndDisplay, cameraIndexL  , std::ref(frameL));
	std::thread thread2(captureAndDisplay, cameraIndexR  , std::ref(frameR));
	std::string imgNameL, imgNameR;
	int f = 1;
	int ch;
	while (1)
	{
		if (_kbhit())
		{
			ch = _getch();
			if (ch == 65) {  //A
				imgNameL = std::to_string(f);
				imgNameL = PathL + imgNameL;
				cv::imwrite(imgNameL + "_L.jpg", frameL);
				imgNameR = std::to_string(f);
				imgNameR = PathR + imgNameR;
				cv::imwrite(imgNameR + "_R.jpg", frameR);
				f++;
			}
			if (ch == 27) // ESC 
			{
				stopFlag = 1;
				break;
			}
		}

	
	}


	thread1.join();
	thread2.join();

	return 0;
}