#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <conio.h>

#define cameraIndex 0
//press "A" to capture imgae

void findAvailableCameras() {
	std::vector<int> available_cameras;
	for (int i = 0; i < 10; ++i) {  
		cv::VideoCapture cap(i);
		if (!cap.isOpened()) {
			continue;
		}
		else {		
			available_cameras.push_back(i);
			cap.release();  
		}
	}

	if (!available_cameras.empty()) {
		std::cout << "cameraAvailable:" << std::endl;
		for (int camera : available_cameras) {
			std::cout << "camera " << camera << std::endl;
		}
	}
	else {
		std::cout << "Can't find any camera" << std::endl;
	}
}

void checkCameraProp(int camIndex) 
{

};

int main() {
	findAvailableCameras();

	cv::VideoCapture video_capture;
	video_capture.open(cameraIndex );
	//video_capture.open(cameraIndex + cv::CAP_DSHOW);


	//cv::VideoWriter writer("VideoTest_2.avi", CV_FOURCC('M', 'P', '4', '2'), 20.0, cv::Size(720, 1280));
	if(video_capture.isOpened())
	{
	video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//640
	video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);//480
	}
	if (!video_capture.isOpened()) {
		std::cout << "failed to capture!" << std::endl;
		return -1;
	}

	std::string imgName;
	int f = 1;
	int ch;

	while (1)
	{
		cv::Mat frame;
		video_capture >> frame;
		//writer << frame;  
		//
		transpose(frame, frame);
		//cv::flip(frame, frame, 0);

		if (frame.empty())
		{
			break;
		}
		cv::namedWindow("win", 0);
		//cv::resizeWindow("win", cv::Size(720 * 0.6, 1280 * 0.6));
		//cv::resizeWindow("win", cv::Size(1280 * 0.6, 720 * 0.6));
		cv::imshow("win", frame);
		if (_kbhit())
		{
			ch = _getch();
			if (ch == 65) {
				imgName = std::to_string(f++);
				cv::imwrite(imgName + ".jpg", frame);
			}
		}
		char key = cv::waitKey(30);
	}
	return 0;
}
