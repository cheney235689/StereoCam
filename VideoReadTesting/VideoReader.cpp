#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <conio.h>

int main() {
	cv::VideoCapture cap("960720.mp4");
    // cap is the object of class video capture that tries to capture Bumpy.mp4
    if (!cap.isOpened())  // isOpened() returns true if capturing has been initialized.
    {
        std::cout << "Cannot open the video file. \n";
        return -1;
    }

    //double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    //// The function get is used to derive a property from the element.
    //// Example:
    //// CV_CAP_PROP_POS_MSEC :  Current Video capture timestamp.
    //// CV_CAP_PROP_POS_FRAMES : Index of the next frame.

    cv::namedWindow("A_good_name", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    // first argument: name of the window.
    // second argument: flag- types: 
    // WINDOW_NORMAL : The user can resize the window.
    // WINDOW_AUTOSIZE : The window size is automatically adjusted to fit the displayed image() ), and you cannot change the window size manually.
    // WINDOW_OPENGL : The window will be created with OpenGL support.
    int frame_counter = 0;
    while (1)
    {
        cv::Mat frame = cv::Mat(720, 960, CV_8UC3, cv::Scalar(0, 0, 0));
        // Mat object is a basic image container. frame is an object of Mat.
        //bool su = cap.read(frame);
        if (!cap.read(frame)) // if not success, break loop
        // read() decodes and captures the next frame.
        {
            std::cout << "\n Cannot read the video file. \n";
            break;
        }

        frame_counter += 1;

        if (frame_counter == int(cap.get(cv::CAP_PROP_FRAME_COUNT))) {
            frame_counter = 0;
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        }


        cv::imshow("A_good_name", frame);
        // first argument: name of the window.
        // second argument: image to be shown(Mat object).

        if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
        {
            break;
        }
    }

    return 0;
}


