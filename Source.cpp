








#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/ximgproc.hpp>

using namespace cv;
using namespace std;





int main(int argc, char** argv)
{
    Mat img1, img2, dst1, dst2, lft, rgt;
    lft = imread("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\omnidirimage\\Left\\left_stid1_frm95160_bid1.bmp", IMREAD_COLOR);
    rgt = imread("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\omnidirimage\\Right\\right_stid1_frm95160_bid2.bmp", IMREAD_COLOR);

    std::vector<double>* lintrin_arr = new std::vector<double>{ 6617.832736895683, 0, 551.3149037466272, 0, 6653.690602126069, 556.9070966245184, 0, 0, 1 };
    std::vector<double>* rintrin_arr = new std::vector<double>{ 6865.490775742195, 0, 679.9317259044155, 0, 6931.301017729188, 585.2658556698447, 0, 0, 1 };
    cv::Mat lk = cv::Mat(3, 3, CV_64F, lintrin_arr->data());
    cv::Mat rk = cv::Mat(3, 3, CV_64F, rintrin_arr->data());

    Mat rdist(1, 4, cv::DataType<double>::type);
    rdist.at<double>(0, 0) = 0;
    rdist.at<double>(0, 1) = 0;
    rdist.at<double>(0, 2) = -0.08920547497423885;
    rdist.at<double>(0, 3) = -0.08353444664858659;

    Mat lXi(1, 1, cv::DataType<double>::type);
    lXi.at<double>(0, 0) = 7.125969950873118;


    Mat ldist(1, 4, cv::DataType<double>::type);
    ldist.at<double>(0, 0) = 0;
    ldist.at<double>(0, 1) = 0;
    ldist.at<double>(0, 2) = -0.04762313500199899;
    ldist.at<double>(0, 3) = -0.04514176028124323;

    Mat rXi(1, 1, cv::DataType<double>::type);
    rXi.at<double>(0, 0) = 7.496757778551663;

    Mat brvecs(3, 1, cv::DataType<double>::type);
    Mat btvecs(3, 1, cv::DataType<double>::type);
    brvecs.at<double>(0, 0) = 0.02258493181867122;
    brvecs.at<double>(1, 0) = -0.001871371100708675;
    brvecs.at<double>(2, 0) = 0.001595470672660145;

    btvecs.at<double>(0, 0) = 63.03479390789293;
    btvecs.at<double>(1, 0) = 0.08430347221701619;
    btvecs.at<double>(2, 0) = 3.720453315014276;


    print(lk);
    print(rk);
    print(rdist);
    print(ldist);
    print(rXi);
    print(lXi);
    print(brvecs);
    print(btvecs);
   


    //Mat imgSize;
    cv::Size imgSize = lft.size();


    Mat E = Mat::eye(3, 3, cv::DataType<double>::type);

    int flag = (cv::omnidir::CALIB_FIX_SKEW + cv::omnidir::CALIB_FIX_K1 + cv::omnidir::CALIB_FIX_K2);
    cv::Matx33d KNew(imgSize.width / 4, 0, imgSize.width / 2,
                     0, imgSize.height / 4, imgSize.height / 2,
                     0, 0, 1);
    Mat pointCloud;
    cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-8);
    //double rms;
    Mat undistortedl, undistortedr;
    cout << "UndistortingL" << endl;
    cv::Matx33d Knew(imgSize.width / 3.1415, 0, 0,
        0, imgSize.height / 3.1415, 0,
        0, 0, 1);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\lft.png", lft);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\rgt.png", rgt);
    omnidir::undistortImage(lft, undistortedl, lk, ldist, lXi, cv::omnidir::RECTIFY_PERSPECTIVE, KNew, imgSize, E);
    omnidir::undistortImage(rgt, undistortedr, rk, rdist, rXi, cv::omnidir::RECTIFY_PERSPECTIVE, KNew, imgSize, E);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\upl.png", undistortedl);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\upr.png", undistortedr);

    omnidir::undistortImage(lft, undistortedl, lk, ldist, lXi, cv::omnidir::RECTIFY_CYLINDRICAL, KNew, imgSize, E);
    omnidir::undistortImage(rgt, undistortedr, rk, rdist, rXi, cv::omnidir::RECTIFY_CYLINDRICAL, Knew, imgSize, E);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\uCl.png", undistortedl);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\uCr.png", undistortedr);

    omnidir::undistortImage(lft, undistortedl, lk, ldist, lXi, cv::omnidir::RECTIFY_LONGLATI, Knew, imgSize, E);
    omnidir::undistortImage(rgt, undistortedr, rk, rdist, rXi, cv::omnidir::RECTIFY_LONGLATI, Knew, imgSize, E);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\uLOl.png", undistortedl);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\uLOr.png", undistortedr);


    int numDisparities = 16 * 8;
    int SADWindowSize = 5;
    cv::Mat disMap;
    int pointType = omnidir::XYZRGB;
    Mat imageRec1, imageRec2;
    
    cv::omnidir::stereoReconstruct(lft, rgt, lk, ldist, lXi, rk, rdist, rXi,brvecs, btvecs, cv::omnidir::RECTIFY_LONGLATI, numDisparities, SADWindowSize, disMap, imageRec1, imageRec2, cv::Size(1152, 1152), Knew, pointCloud, pointType);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\disMapLongLa.png", disMap);

    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\irl.png", imageRec1);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\irr.png", imageRec2);
    cv::omnidir::stereoReconstruct(lft, rgt, lk, ldist, lXi, rk, rdist, rXi, brvecs, btvecs, cv::omnidir::RECTIFY_PERSPECTIVE, numDisparities, SADWindowSize, disMap, imageRec1, imageRec2, cv::Size(1152, 1152), KNew, pointCloud, pointType);
    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\disMapPers.png", disMap);


    cv::Mat map1, map2, map3, map4, undistort;

    int blockSize = 5 * 2 + 5;
    int preFilterType = 1;
    int preFilterSize = 1*2+5;
    int preFilterCap = 31;
    int minDisparity = 0;
    int textureThreshold = 10;
    int uniquenessRatio = 15;
    int speckleRange = 0;
    int speckleWindowSize = 0*2;
    int disp12MaxDiff = -1;
    int dispType = CV_16S;

    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

    Mat imgL_gray, imgr_gray, disp,dispConvert;
    cv::cvtColor(imageRec1, imgL_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imageRec2, imgr_gray, cv::COLOR_BGR2GRAY);
    stereo->setNumDisparities(numDisparities);
    stereo->setBlockSize(blockSize);
    stereo->setPreFilterSize(preFilterSize);
    stereo->setPreFilterCap(preFilterCap);
    stereo->setTextureThreshold(textureThreshold);
    stereo->setUniquenessRatio(uniquenessRatio);
    stereo->setSpeckleRange(speckleRange);
    stereo->setSpeckleWindowSize(speckleWindowSize);
    stereo->setDisp12MaxDiff(disp12MaxDiff);
    stereo->setMinDisparity(minDisparity);
    stereo->compute(imgL_gray, imgr_gray, disp);
    disp.convertTo(dispConvert, CV_32F, 1.0);
    dispConvert = (dispConvert / 16.0f - (float)minDisparity) / ((float)numDisparities);
    imshow("Disparityconvert", dispConvert);

    imwrite("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\results\\disMapsgbm.jpg", dispConvert);



    //cv::omnidir::stereoReconstruct(dst1, dst2, lk, ldist, lXi, rk, rdist, rXi, R, T, flag, numDisparities, SADWindowSize, disMap, imageRec1, imageRec2, imgSize, KNew, pointCloud);

    //omnidir::initUndistortRectifyMap(lk, ldist, lXi, lextrinsics, KN-ew, imgSize, CV_16SC2, map1, map2, flag);
    //cv::remap(lft, dst1, map1, map2, INTER_LINEAR, BORDER_CONSTANT);


    //cv::fisheye::initUndistortRectifyMap(lk, ldist, E, lk, imgSize, CV_16SC2, map3, map4);
    //cv::remap(lft, undistort, map3, map4, INTER_LINEAR, CV_HAL_BORDER_CONSTANT);

    //cv::omnidir::stereoReconstruct(lft, rgt, lk, ldist, lXi, rk, rdist, rXi, lRotVec, ltVect, flag, numDisparities, SADWindowSize, disMap, dst1, dst2, imgSize, KNew, pointCloud);
   // cv::omnidir::undistortImage(lft,dst1,lk,ldist,lXi, cv::omnidir::RECTIFY_PERSPECTIVE,KNew,imgSize,lRotVec);

    //cv::fisheye::undistortImage(lft, undistort, lk, ldist, lk, imgSize);


   

    //show disparity image;
    /*
    cv::namedWindow("Disparity");
    cv::imshow("Disparity",disparity_image);
    cv::waitKey(0);
    return 0;*/




    namedWindow("lft 1", WINDOW_AUTOSIZE); // Create a window for display.
    imshow("lft 1", undistortedl); // Show our image inside it.

    namedWindow("Result 1", WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Result 1", imageRec1);

    namedWindow("Result 2", WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Result 2", imageRec2);
    namedWindow("disparity", WINDOW_AUTOSIZE);
    imshow("disparity", disMap);
    namedWindow("POINTCLOUD", WINDOW_AUTOSIZE);
    //imshow("POINTCLOUD", pointCloud);
    waitKey(0); // Wait for a keystroke in the window
    return 0;



}
