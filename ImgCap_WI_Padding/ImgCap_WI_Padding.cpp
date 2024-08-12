#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <conio.h>

int main() {
	int top, bottom, left, right;
	float ratio = 1.0; 
	top = (int)(480* ratio);
	bottom = (int)(480* ratio);
	left = (int)(640* ratio);
	right = (int)(640* ratio);
	cv::RNG rng(12345);

	cv::VideoCapture video_capture(0);
	cv::VideoWriter writer("VideoTest_2.avi", CV_FOURCC('M', 'P', '4', '2'), 20.0, cv::Size(640 , 480 ));
	video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);//640
	video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 640);//480
	if (!video_capture.isOpened()) {
		std::cout << "failed to capture!" << std::endl;
		return -1;
	}

	std::string imgName;
	int f = 1;
	int ch;
	cv::Scalar value(rng.uniform(254, 255), rng.uniform(254, 255), rng.uniform(254, 255));
	while (1)
	{
		cv::Mat frame;
		video_capture >> frame;
		cv::copyMakeBorder(frame, frame, top, bottom, left, right, cv::BORDER_CONSTANT, value);
		cv::resize(frame, frame, cv::Size(640, 480));
		writer << frame;


		if (frame.empty()) break;
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