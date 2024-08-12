
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

double pi = 3.14159265359;
double conv = 180 / pi;
#define cameraIndexG 0
#define cameraIndexR 2

using namespace cv;



cv::Mat Q;
cv::Mat xyz;  //3D_point


//cv::Mat cameraMatrix_Left = (cv::Mat1d(3, 3) << 467.4054845351391, 0, 345.3676116667228,
//    0, 466.1525243930371, 258.1326135651114,
//    0, 0, 1);
//cv::Mat cameraMatrix_Right = (cv::Mat1d(3, 3) << 484.8126541197303, 0, 301.5035386755649,
//    0, 484.7594965367413, 248.7231220574427,
//    0, 0, 1);
//cv::Mat distortionCoefficients_Left = (cv::Mat1d(1, 5) << -0.427635865009219, 0.4062983471869417, -0.004090892521349689, -0.002149580696683241, -0.4257508378178044);
//cv::Mat distortionCoefficients_Right = (cv::Mat1d(1, 5) << -0.3883301409652372, 0.01171919738593649, 0.001104793189431507, -0.00159489954108481, 0.467298086898806);



cv::Vec3d rotationMatrixToEulerAngles(cv::Mat& R)
{

    assert(isRotationMatrix(R));

    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    double x, y, z;
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
    return cv::Vec3d(x, y, z);

}


struct CamIntrinsic
{
    double cx, cy, fx, fy, baseline, scale_factor;
};

#if 0
void calDisparity_testing(const IplImage* left, const IplImage* right, Mat& disparity)
{
    Mat _left = cvarrToMat(left); 
    Mat _right = cvarrToMat(right);
    Rect leftROI, rightROI;
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);
    bm->setPreFilterType(CV_STEREO_BM_XSOBEL);  
    bm->setPreFilterSize(9);
    bm->setPreFilterCap(31);
    bm->setBlockSize(15);
    bm->setMinDisparity(0);
    bm->setNumDisparities(64);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(5);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setROI1(leftROI);
    bm->setROI2(rightROI);
    copyMakeBorder(_left, _left, 0, 0, 80, 0, IPL_BORDER_REPLICATE);  
    copyMakeBorder(_right, _right, 0, 0, 80, 0, IPL_BORDER_REPLICATE);
    bm->compute(_left, _right, disparity);
    disparity = disparity.colRange(80, _left.cols);
    disparity.convertTo(disparity, CV_32F, 1.0 / 16);
}
#endif // 0



void calDisparity( Mat _left,  Mat _right, Mat& disparity)
{
    cvtColor(_right, _right, CV_RGB2GRAY);
    cvtColor(_left, _left, CV_RGB2GRAY);
    std::cout << "gray_left.depth()  : " << _left.depth() << std::endl;
    std::cout << "gray_left.channels()  : " << _left.channels() << std::endl;
    std::cout << "gray_left.elemSize()  : " << _left.elemSize() << std::endl;
    cv:Rect leftROI, rightROI;
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);
    bm->setPreFilterType(CV_STEREO_BM_XSOBEL);
    bm->setPreFilterSize(9);
    bm->setPreFilterCap(31);
    bm->setBlockSize(15);
    bm->setMinDisparity(0);
    bm->setNumDisparities(64);//64
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(5);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setUniquenessRatio(5);
    bm->setDisp12MaxDiff(1);
    //bm->setROI1(leftROI);
    //bm->setROI2(rightROI);
    //copyMakeBorder(_left, _left, 0, 0, 80, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(_right, _right, 0, 0, 80, 0, IPL_BORDER_REPLICATE);
    bm->compute(_left, _right, disparity);  // -2^15 ~ +2^15
    std::cout << "disparity.dims ()  : " << disparity.dims << std::endl;
    std::cout << "disparity.depth()  : " << disparity.depth() << std::endl;
    std::cout << "disparity.channels()  : " << disparity.channels() << std::endl;
    std::cout << "disparity.elemSize()  : " << disparity.elemSize() << std::endl;
    std::cout << "disparity.elemSize1()  : " << disparity.elemSize1() << std::endl;
    //disparity = disparity.colRange(80, _left.cols);
    //disparity.convertTo(disparity, CV_32F, 1.0 / 16);   //<-ori
    
    //disparity.convertTo(disparity, CV_8UC1, 1.0 / 256   );
    //disparity.convertTo(disparity, CV_8UC1, 255 /(64 * 16.) );


}

void disparity2depth(Mat disparity , Mat &depth , CamIntrinsic cam)
{


    depth.create(disparity.rows , disparity.cols , CV_8UC1);

    Mat _depth = Mat(disparity.rows, disparity.cols, CV_16S);
    for (int i = 0 ; i <disparity.rows  ; i++)
    {
        for (int j = 0; j < disparity.cols; j++)
        {
            //disparity.ptr<ushort>(i)[j] = disparity.ptr<ushort>(i)[j] + 1;

            if (!disparity.ptr<ushort>(i)[j])
                continue;
            _depth.ptr<ushort>(i)[j] = cam.scale_factor * sqrt(cam.fx * cam.fx + cam.fy * cam.fy) * cam.baseline / disparity.ptr<ushort>(i)[j];
        
        }
    }
    _depth.convertTo(depth , CV_8U , 1./256);
}
void disparity2depth_v2(Mat disparity, Mat& depth, CamIntrinsic cam)
{

    depth.create(disparity.rows, disparity.cols, CV_8UC1);

    //disparity.convertTo(disparity, 1, 1);

    Mat _depth = Mat(disparity.rows, disparity.cols, CV_16S);

    for (int i = 0; i < disparity.rows; i++)
    {
        for (int j = 0; j < disparity.cols; j++)
        {
            
            _depth.ptr<ushort>(i)[j] = cam.scale_factor * sqrt(cam.fx * cam.fx + cam.fy * cam.fy) * cam.baseline / disparity.ptr<ushort>(i)[j];
            //_depth.ptr<ushort>(i)[j] = disparity.ptr<ushort>(i)[j]/cam.scale_factor * sqrt(cam.fx * cam.fx + cam.fy * cam.fy) * cam.baseline ;


        }
    }
    _depth.convertTo(depth, CV_8U);
}






//double meamdisp(Mat disparity)
//{
//    double sum = 0;
//    for (int i = 0; i < disparity.rows; i++)
//    {
//        for (int j = 0; j < disparity.cols; j++)
//        {
//            if (!(disparity.ptr<ushort>(i)[j]))
//            {
//                sum = sum + 1;
//            }
// 
//        }
//    }
//    return sum;
//}




int main(){
    //Cam streaming
    cv::VideoCapture video_capture_1(0);
    cv::VideoCapture video_capture_2(1);
    video_capture_1.set(cv::CAP_PROP_FRAME_WIDTH, 640);//640
    video_capture_1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);//480
    video_capture_2.set(cv::CAP_PROP_FRAME_WIDTH, 640);//640
    video_capture_2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);//480
    cv::Size Imgsize = cv::Size(640 , 480);

    cv::Mat R1, R2, P1, P2  ,  L_map1, L_map2 , R_map1, R_map2;
    //Rotation vector : [0.9999829912845903, -0.005831025141867779, 0.0001276217764369169;
    //0.005831030576238699, 0.9999829985040994, -4.225126385794918e-05;
    //-0.0001273732384939765, 4.299471169882429e-05, 0.9999999909637564]
    cv::Mat rotation_matrix = cv::Mat::zeros(3,3,CV_64F);
    rotation_matrix.at<double>(0, 0) = 0.9999829912845903;
    rotation_matrix.at<double>(0, 1) = -0.005831025141867779;
    rotation_matrix.at<double>(0, 2) = 0.0001276217764369169;
    rotation_matrix.at<double>(1, 0) = 0.005831030576238699;
    rotation_matrix.at<double>(1, 1) = 0.9999829985040994;
    rotation_matrix.at<double>(1, 2) = -4.225126385794918e-05;
    rotation_matrix.at<double>(2, 0) = -0.0001273732384939765;
    rotation_matrix.at<double>(2, 1) = 4.299471169882429e-05;
    rotation_matrix.at<double>(2, 2) = 0.9999999909637564;

    cv::Mat rotation_vector ;
    cv::Rodrigues(rotation_matrix ,rotation_vector);
    //Translation vector : [-264.947, -9.17059, 0.0627383]
    cv::Mat translation_vector = cv::Mat::zeros(3,1,CV_64F);
    translation_vector.at<double>(0, 0) = -264.947;
    translation_vector.at<double>(1, 0) = -9.17059;
    translation_vector.at<double>(2, 0) = 0.0627383;

    cv::Mat cameraMatrix_Left = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrix_Left.at<double>(0, 0) = 467.4054845351391; //fx
    cameraMatrix_Left.at<double>(0, 1) = 0; //s
    cameraMatrix_Left.at<double>(0, 2) = 345.3676116667228; //cx
    cameraMatrix_Left.at<double>(1, 1) = 466.1525243930371; //fy
    cameraMatrix_Left.at<double>(1, 2) = 258.1326135651114;//cy
    cameraMatrix_Left.at<double>(2, 2) = 1;

    cv::Mat cameraMatrix_Right = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrix_Right.at<double>(0, 0) = 484.8126541197303; //fx
    cameraMatrix_Right.at<double>(0, 1) = 0; //s
    cameraMatrix_Right.at<double>(0, 2) = 301.5035386755649; //cx
    cameraMatrix_Right.at<double>(1, 1) = 484.7594965367413; //fy
    cameraMatrix_Right.at<double>(1, 2) = 258.1326135651114;//cy
    cameraMatrix_Right.at<double>(2, 2) = 1;
    
    cv::Mat distortionCoefficients_Left = cv::Mat::zeros(5, 1, CV_64F);
    distortionCoefficients_Left.at<double>(0, 0) = -0.427635865009219;
    distortionCoefficients_Left.at<double>(1, 0) = 0.4062983471869417;
    distortionCoefficients_Left.at<double>(2, 0) = -0.004090892521349689;
    distortionCoefficients_Left.at<double>(3, 0) = -0.002149580696683241;
    distortionCoefficients_Left.at<double>(4, 0) = -0.4257508378178044;

    cv::Mat distortionCoefficients_Right = cv::Mat::zeros(5, 1, CV_64F);
    distortionCoefficients_Right.at<double>(0, 0) = -0.3883301409652372;
    distortionCoefficients_Right.at<double>(1, 0) = 0.01171919738593649;
    distortionCoefficients_Right.at<double>(2, 0) = 0.001104793189431507;
    distortionCoefficients_Right.at<double>(3, 0) = -0.00159489954108481;
    distortionCoefficients_Right.at<double>(4, 0) = 0.467298086898806;


    cv::stereoRectify(cameraMatrix_Left , distortionCoefficients_Left , cameraMatrix_Right , distortionCoefficients_Right,
                    Imgsize , rotation_vector , translation_vector , 
                    R1, R2, P1, P2 , Q , 
                    CALIB_ZERO_DISPARITY , -1 , Imgsize );
    //Rot. , Ttans. (L->R)
    //R1 (1->1_prime)
    //R2 (2->2_prime)
    //P1 , P2  (coord. is reference to cam1 (left))

    //std::cout << "R1 : " << R1 << std::endl;
    //std::cout << "R2 : " << R2 << std::endl;
    //std::cout << "P1  : " << P1 << std::endl;
    //std::cout << "P2  : " << P2 << std::endl;

    //cv::Vec3d r0EulerV  = rotationMatrixToEulerAngles(rotation_matrix);
    //r0EulerV = r0EulerV * conv;
    //cv::Vec3d r1EulerV = rotationMatrixToEulerAngles(R1);
    //r1EulerV = r1EulerV * conv;
    //cv::Vec3d r2EulerV = rotationMatrixToEulerAngles(R2);
    //r2EulerV = r2EulerV * conv;

    //std::cout << "r0EulerV : " << r0EulerV << std::endl;
    //std::cout << "r1EulerV  : " << r1EulerV << std::endl;
    //std::cout << "r2EulerV  : " << r2EulerV << std::endl;


    cv::initUndistortRectifyMap(cameraMatrix_Left , distortionCoefficients_Left ,R1 , P1 , Imgsize , CV_16SC2,L_map1,L_map2);//L
    cv::initUndistortRectifyMap(cameraMatrix_Right, distortionCoefficients_Right, R2, P2, Imgsize, CV_16SC2, R_map1, R_map2);//R
    //std::cout << "L_map1 size: " << L_map1.size() << std::endl;
    //std::cout << "L_map2  size: " << L_map2.size() << std::endl;
    //std::cout << "R_map1  size: " << R_map1.size() << std::endl;
    //std::cout << "R_map2  size: " << R_map2.size() << std::endl;
    

    //Mat imgL = imread("left.jpg",  0);
	//Mat imgR = imread("right.jpg", 0);
    CamIntrinsic cam;
    cam.baseline = sqrt(264.947 * 264.947 + 9.17059 * 9.17059 + 0.0627383 * 0.0627383);
    //cam.baseline = 25;

    cam.fx = 467.4054845351391;
    cam.fy = 345.3676116667228;
    cam.scale_factor = 1.0;
    cam.cx = 466.1525243930371;
    cam.cy = 258.1326135651114;



    


    while (true)
    {
    //cv::Mat depth_result, depth_result_color;

    cv::Mat imgL , imgL_Rectified;
    video_capture_1 >> imgL;
    cv::Mat imgR , imgR_Rectified;
    video_capture_2 >> imgR;
    std::cout << "imgL.depth()  : " << imgL.depth() << std::endl;
    std::cout << "imgL.channels()  : " << imgL.channels() << std::endl;
    std::cout << "imgL.elemSize()  : " << imgL.elemSize() << std::endl;
    cv::imshow("left_cam", imgL);
    //cv::imshow("right_cam", imgR);
    cv::remap(imgL, imgL_Rectified, L_map1, L_map2, INTER_LINEAR);//L
    cv::remap(imgR, imgR_Rectified, R_map1, R_map2, INTER_LINEAR);//R  
    //Mat disparity;
    //calDisparity(imgL, imgR, disparity);
    //imshow("raw_disp_map", disparity);
    //bitwise_not(disparity , disparity);
    std::cout << "imgL_Rectified.depth()  : " << imgL_Rectified.depth() << std::endl;
    std::cout << "imgL_Rectified.channels()  : " << imgL_Rectified.channels() << std::endl;
    std::cout << "imgL_Rectified.elemSize()  : " << imgL_Rectified.elemSize() << std::endl;

    cv::imshow("imgL_Rectified", imgL_Rectified);
    //cv::imshow("imgR_Rectified", imgR_Rectified);
    Mat disparity_rectified , disparity_rectified_show, depth_result , depth_result_color;
    calDisparity(imgL_Rectified, imgR_Rectified, disparity_rectified);

    
    disparity_rectified.convertTo(disparity_rectified_show, CV_32F, 1.0 / 16);   //<-ori
    //cv::Mat testing_disparity = cv::Mat(disparity_rectified.rows , disparity_rectified.cols , CV_8U);
    //cv::normalize(disparity_rectified, testing_disparity , 0 , 255 ,NORM_MINMAX,CV_8U);

    std::cout << "disparity_rectified.depth()  : " << disparity_rectified.depth() << std::endl;
    std::cout << "disparity_rectified.channels()  : " << disparity_rectified.channels() << std::endl;
    std::cout << "disparity_rectified.elemSize()  : " << disparity_rectified.elemSize() << std::endl;

    imshow("disparity_rectified_raw", disparity_rectified);
    imshow("disparity_rectified_CV32F_(1/16)", disparity_rectified_show);
    disparity2depth(disparity_rectified , depth_result , cam);

    std::cout << "depth_result.depth()  : " << depth_result.depth() << std::endl;
    std::cout << "depth_result.channels()  : " << depth_result.channels() << std::endl;
    std::cout << "depth_result.elemSize()  : " << depth_result.elemSize() << std::endl;

    cv::applyColorMap(depth_result , depth_result_color , cv::COLORMAP_JET);

    std::cout << "depth_result_color.depth()  : " << depth_result_color.depth() << std::endl;
    std::cout << "depth_result_color.channels()  : " << depth_result_color.channels() << std::endl;
    std::cout << "depth_result_color.elemSize()  : " << depth_result_color.elemSize() << std::endl;


    imshow("depth_result", depth_result);
    imshow("depth_result_color", depth_result_color);
        





    cv::Mat depth_v2;
    disparity2depth_v2(disparity_rectified, depth_v2, cam);
    imshow("depth_v2", depth_v2);
    cv::applyColorMap(depth_v2, depth_v2, cv::COLORMAP_JET);

    imshow("depth_v2_COLOR", depth_v2);



    //double sum = meamdisp(disparity_rectified);
    //std::cout << "sum  : " << sum << std::endl;
    //double ratio = sum / 307200;
    //std::cout << "ratio  : " << ratio << std::endl;
    
    
    //depth_result.release();
    //depth_result_color.release();
    cv::waitKey(20);  //don't comment out!!
	}

	
return 0;
}
//imgL.depth() : 0
//imgL.channels() : 3
//imgL.elemSize() : 3
//imgL_Rectified.depth() : 0
//imgL_Rectified.channels() : 3
//imgL_Rectified.elemSize() : 3
//gray_left.depth() : 0
//gray_left.channels() : 1
//gray_left.elemSize() : 1
// 
// (CV8UC3 -> CV8U)
// 
//disparity.depth() : 3
//disparity.channels() : 1
//disparity.elemSize() : 2
// 
// (CV8U - > CV16S)
// 
//disparity_rectified.depth() : 5
//disparity_rectified.channels() : 1
//disparity_rectified.elemSize() : 4
// 
//depth_result.depth() : 0
//depth_result.channels() : 1
//depth_result.elemSize() : 1
// 
//depth_result_color.depth() : 0
//depth_result_color.channels() : 3
//depth_result_color.elemSize() : 3