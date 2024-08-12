#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#define cameraIndexR 0
#define cameraIndexL 2

#define showVideo
struct CamIntrinsic
{
    double cx, cy, fx, fy, baseline, scale_factor;
};

void SGBM(cv::Mat imgL, cv::Mat imgR, cv::Mat& disparity)
{
    int pBlockSize = 3;
    int pNumDisparities = 16 ;
    int pP1 = 1;
    int pP2 = 8;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, pNumDisparities, pBlockSize);
    sgbm->setPreFilterCap(63);
    sgbm->setBlockSize(pBlockSize);
    sgbm->setP1(pP1 * 1 * pBlockSize * pBlockSize);
    sgbm->setP2(pP2 * 1 * pBlockSize * pBlockSize);
    sgbm->setDisp12MaxDiff(3);
    sgbm->setNumDisparities(pNumDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(0);
    sgbm->setSpeckleRange(32);
    sgbm->compute(imgL, imgR, disparity);

    //cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
    //sgbm->setMinDisparity(0);
    //sgbm->setNumDisparities(128);
    //sgbm->setBlockSize(5);
    //sgbm->setP1(8 * imgL.channels() * sgbm->getBlockSize() * sgbm->getBlockSize());
    //sgbm->setP2(32 * imgL.channels() * sgbm->getBlockSize() * sgbm->getBlockSize());
    //sgbm->compute(imgL, imgR, disparity);

    //std::cout << imgL.channels() << std::endl;


}

void disparity2depth(cv::Mat disparity, cv::Mat& depth, CamIntrinsic cam)
{

    depth.create(disparity.rows, disparity.cols, CV_8UC1);

    //disparity.convertTo(disparity, 1, 1);

    cv::Mat _depth = cv::Mat(disparity.rows, disparity.cols, CV_16S);

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

int main()
{
    cv::Mat rotation_matrix = cv::Mat::zeros(3, 3, CV_64F);
    rotation_matrix.at<double>(0, 0) = 0.9983524073585465;
    rotation_matrix.at<double>(0, 1) = -0.02314301143349438;
    rotation_matrix.at<double>(0, 2) = 0.05250592102976598;
    rotation_matrix.at<double>(1, 0) = -0.01521680527104671;
    rotation_matrix.at<double>(1, 1) = 0.989075734326732;
    rotation_matrix.at<double>(1, 2) = 0.146620737289712;
    rotation_matrix.at<double>(2, 0) = -0.05532557779850038;
    rotation_matrix.at<double>(2, 1) = -0.1455801936659821;
    rotation_matrix.at<double>(2, 2) = 0.9877983031233842;

    cv::Mat translation_vector = cv::Mat::zeros(3, 1, CV_64F);
    translation_vector.at<double>(0, 0) = -318.445;
    translation_vector.at<double>(1, 0) = 11.2334;
    translation_vector.at<double>(2, 0) = 72.7138;

    cv::Mat cameraMatrix_Left = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrix_Left.at<double>(0, 0) = 1023.860555135772; //fx
    cameraMatrix_Left.at<double>(0, 1) = 0; //s
    cameraMatrix_Left.at<double>(0, 2) = 576.6506995877392; //cx
    cameraMatrix_Left.at<double>(1, 1) = 1037.381203701293; //fy
    cameraMatrix_Left.at<double>(1, 2) = 492.1782776367298;//cy
    cameraMatrix_Left.at<double>(2, 2) = 1;

    cv::Mat cameraMatrix_Right = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrix_Right.at<double>(0, 0) = 1082.249149989758; //fx
    cameraMatrix_Right.at<double>(0, 1) = 0; //s
    cameraMatrix_Right.at<double>(0, 2) = 597.7924745269019; //cx
    cameraMatrix_Right.at<double>(1, 1) = 1081.607999307285; //fy
    cameraMatrix_Right.at<double>(1, 2) = 380.9839572986523;//cy
    cameraMatrix_Right.at<double>(2, 2) = 1;

    cv::Mat distortionCoefficients_Left = cv::Mat::zeros(5, 1, CV_64F);
    distortionCoefficients_Left.at<double>(0, 0) = 0.05272633126360989;
    distortionCoefficients_Left.at<double>(1, 0) = -0.05943827780059409;
    distortionCoefficients_Left.at<double>(2, 0) = 0.03621271857005092;
    distortionCoefficients_Left.at<double>(3, 0) = -0.001091206292700157;
    distortionCoefficients_Left.at<double>(4, 0) = -0.07631643607320954;

    cv::Mat distortionCoefficients_Right = cv::Mat::zeros(5, 1, CV_64F);
    distortionCoefficients_Right.at<double>(0, 0) = -0.09863898917440568;
    distortionCoefficients_Right.at<double>(1, 0) = 0.07605852824232937;
    distortionCoefficients_Right.at<double>(2, 0) = -0.09204349608940217;
    distortionCoefficients_Right.at<double>(3, 0) = 0.02328603609772354;
    distortionCoefficients_Right.at<double>(4, 0) = 0.2624082764919622;


    CamIntrinsic cam;

    cam.baseline = sqrt((-318.445) * (-318.445) + 11.2334 * 11.2334 + 72.7138 * 72.7138);

    //cam.fx = 1023.860555135772;
    //cam.fy = 576.6506995877392;
    //cam.scale_factor = 1.0;
    //cam.cx = 1037.381203701293;
    //cam.cy = 492.1782776367298;

    cam.fx = 1082.249149989758;
    cam.fy = 597.7924745269019;
    cam.scale_factor = 1.0;
    cam.cx = 1081.607999307285;
    cam.cy = 380.9839572986523;




    int stereoRectifyFlag = 0;

    double balance = 0.0;
    double fovScale = 1.0;
    cv::Mat Q;
    cv::Mat R1, R2, P1, P2;
    cv::Size Imgsize = cv::Size(1280*1.3, 720*1.3);
    cv::stereoRectify(cameraMatrix_Left, distortionCoefficients_Left, cameraMatrix_Right, distortionCoefficients_Right,
        Imgsize, rotation_matrix, translation_vector,
        R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, -1, Imgsize);

    cv::Mat L_map1, L_map2, R_map1, R_map2;
    cv::initUndistortRectifyMap(cameraMatrix_Left , distortionCoefficients_Left ,R1 , P1 , Imgsize , CV_16SC2,L_map1,L_map2);//L
    cv::initUndistortRectifyMap(cameraMatrix_Right, distortionCoefficients_Right, R2, P2, Imgsize, CV_16SC2, R_map1, R_map2);//R


    //cam streaming
    cv::VideoCapture videoCaptureRight(cameraIndexR);
    cv::VideoCapture videoCaptureLeft(cameraIndexL);
    videoCaptureRight.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//640
    videoCaptureRight.set(cv::CAP_PROP_FRAME_HEIGHT, 720);//480
    videoCaptureLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//640
    videoCaptureLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 720);//480

    while (true)
    {
#ifdef showVideo
        cv::Mat imgL, imgL_Rectified;
        videoCaptureLeft >> imgL;
        //transpose(imgL, imgL);
        //cv::flip(imgL, imgL, 0);
        cv::namedWindow("leftCamOri", 0);
        cv::resizeWindow("leftCamOri", cv::Size(1280 * 0.5, 720 * 0.5));
        cv::imshow("leftCamOri", imgL);


        cv::Mat imgR, imgR_Rectified;
        videoCaptureRight >> imgR;
        //transpose(imgR, imgR);
        //cv::flip(imgR, imgR, 0);
        cv::namedWindow("rightCamOri", 0);
        cv::resizeWindow("rightCamOri", cv::Size(1280 * 0.5, 720 * 0.5));
        cv::imshow("rightCamOri", imgR);

        cv::remap(imgL, imgL_Rectified, L_map1, L_map2, cv::INTER_LINEAR);//L
        cv::remap(imgR, imgR_Rectified, R_map1, R_map2, cv::INTER_LINEAR);//R  

       
        
        cv::namedWindow("leftCamRect", 0);
        cv::resizeWindow("leftCamRect", cv::Size(1280 * 0.5, 720 * 0.5));
        cv::imshow("leftCamRect", imgL_Rectified);
        
        
        cv::namedWindow("rightCamRect", 0);
        cv::resizeWindow("rightCamRect", cv::Size(1280 * 0.5, 720 * 0.5));
        cv::imshow("rightCamRect", imgR_Rectified);




        cv::Mat disparity;
        cv::Mat L_gray, R_gray;
        //SGBM(imgL_Rectified, imgR_Rectified, disparity);

        cv::cvtColor(imgL_Rectified, L_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR_Rectified, R_gray, cv::COLOR_BGR2GRAY);
        SGBM(L_gray, R_gray, disparity);
        cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8U);

        //cv::cvtColor(imgL, L_gray, cv::COLOR_BGR2GRAY);
        //cv::cvtColor(imgR, R_gray, cv::COLOR_BGR2GRAY);
        //SGBM(L_gray, R_gray, disparity);
        //cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8U);

        cv::namedWindow("disparity", 0);
        cv::resizeWindow("disparity", cv::Size(1280 * 0.5, 720 * 0.5));
        cv::imshow("disparity", disparity);

        //cv::Mat disparityScaled;
        //disparity.convertTo(disparityScaled, CV_32F, 1.0 / 16.0);
        //cv::imshow("disparityScaled", disparityScaled);

        //cv::Mat disparityColor;
        //cv::applyColorMap(disparityScaled, disparityColor, cv::COLORMAP_JET);
        //cv::imshow("disparityColor", disparityColor);

        //cv::Mat depthMap;
        //cv::reprojectImageTo3D(disparity, depthMap, Q, false, CV_32F);
        //cv::imshow("depthMap", depthMap);

        cv::Mat depthReal;
        disparity2depth(disparity, depthReal, cam);

        cv::namedWindow("depthReal", 0);
        cv::resizeWindow("depthReal", cv::Size(1280 * 0.5, 720 * 0.5));
        cv::imshow("depthReal", depthReal);

        cv::applyColorMap(depthReal, depthReal, cv::COLORMAP_JET);

        cv::namedWindow("depth_COLOR", 0);
        cv::resizeWindow("depth_COLOR", cv::Size(1280 * 0.5, 720 * 0.5));
        cv::imshow("depth_COLOR", depthReal);

#endif // showVideo

#ifndef showVideo
        cv::Mat imgR = cv::imread("R.jpg");
        //transpose(imgR, imgR);
        //cv::flip(imgR, imgR, 0);
        cv::imshow("rightImgOri", imgR);


        cv::Mat imgL = cv::imread("L.jpg");
        //transpose(imgL, imgL);
        //cv::flip(imgL, imgL, 0);
        cv::imshow("leftImgOri", imgL);


        cv::Mat imgL_Rectified, imgR_Rectified;
        cv::remap(imgL, imgL_Rectified, L_map1, L_map2, cv::INTER_LINEAR);//L
        cv::remap(imgR, imgR_Rectified, R_map1, R_map2, cv::INTER_LINEAR);//R  
        cv::imshow("leftImgRect", imgL_Rectified);
        cv::imshow("rightImgRect", imgR_Rectified);

        cv::Mat disparity;
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16 * 5, 21);
        sgbm->setPreFilterCap(63);
        sgbm->setBlockSize(21);
        sgbm->setP1(8 * 1 * 21 * 21);
        sgbm->setP2(32 * 1 * 21 * 21);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(16 * 5);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->compute(imgR_Rectified, imgL_Rectified, disparity);

        cv::imshow("disparity", disparity);

        cv::Mat disparityScaled;
        disparity.convertTo(disparityScaled, CV_32F, 1.0 / 16.0);
        cv::imshow("disparityScaled", disparityScaled);
#endif // !showVideo

        cv::waitKey(20);
    }

}