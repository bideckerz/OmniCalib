#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;
 


int main(int argc, char** argv)
{


    Mat img1, img2, dst1, dst2, lft, rgt;
    img1 = imread("C:/Users/acevedo/Documents/Visual Studio 2019/data/artroom1/im0.png", IMREAD_GRAYSCALE); // Read the file
    img2 = imread("C:/Users/acevedo/Documents/Visual Studio 2019/data/artroom1/im1.png", IMREAD_GRAYSCALE); // Read the file
    dst1 = imread("C:/Users/acevedo/Documents/Visual Studio 2019/data/artroom1/disp0.pfm", IMREAD_GRAYSCALE); // Read the file
    lft = imread("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\views\\left_stid1_frm341877_bid1.bmp", IMREAD_GRAYSCALE);
    rgt = imread("C:\\Users\\acevedo\\Documents\\Visual Studio 2019\\Varjo\\OpenCV\\views\\right_stid1_frm341877_bid2.bmp", IMREAD_GRAYSCALE);
    double ldoffs[6] = { -0.8656,0.475532,0,0.618565,-0.000908928,0.00113989 };
    double lExtrinsics = ((0.999996, 0.001212, 0.002608, 0.000000), (-0.001199, 0.999988, -0.004693, 0.000000), (-0.002614, 0.004690, 0.999986, 0.000000), (0.000000, 0.000000, 0.000000, 1.000000));
    double lflenx = 1.18979;
    double lleny = 1.18979;
    double lpPointX = 0.535712;
    double lpPointY = 0.0499589;

    double rDoffs[6] = { -0.902073, 0.514595, 0, 0.656539, -0.000661755, 0.000291338 };
    double rExtrinsics = ((0.999939, 0.000000, 0.011042, 0.000000), (-0.000052, 0.999989, 0.004690, 0.000000), (-0.011042, -0.004690, 0.999928, 0.000000), (-0.063388, 0.000000, -0.000700, 1.000000));
    double rflenx = 1.18979;
    double rleny = 1.18979;
    double rpPointX = 0.535712;
    double rpPointY = 0.0499589;
    
    
    
    //Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 190, 1, 600, 2400, -1, 16, 2, 20, 30, true);

    //ouble minVal; double maxVal;


    //sgbm->compute(img1, img2, img1);
    //minMaxLoc(dst1, &minVal, &maxVal);
    //dst1.convertTo(dst2, CV_8UC1, 255 / (maxVal - minVal));

    


    //dst1 = imread("C:/Users/acevedo/Documents/Visual Studio 2019/Varjo/OpenCV/disp0.pfm", IMREAD_UNCHANGED); // Read the file

    //img2=loadPFM("C:/Users/acevedo/Documents/Visual Studio 2019/Varjo/OpenCV/disp0.pfm");

    
    //normalize(img1, dst1, 0, 255, NORM_MINMAX);
    //normalize(img2, dst2, 0, 255, NORM_MINMAX,CV_8U);

    //normalize(img1, dst1, 0, 255, NORM_MINMAX,-1,noArray());

    int kernelsize = 3;
    int doffs = 85;
    float top = 529.50* 1734.04;
    //dst1 = top / img1;
    int offset = 190;

    /*
    for (int y = 0; y < dst1.rows; ++y) {
        for (int x = 0; x < dst1.cols; ++x) {

            dst2.at<float>(y, x) = top / (dst1.at<float>(y, x)*85);
            dst1.at<float>(y, x) = (dst1.at<float>(y, x) - dst1.at<float>(y, x));

        }
    }
    */

 








    namedWindow("Right 1", WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Right 1", rgt); // Show our image inside it.

    namedWindow("Left 2", WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Left 2", lft); // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window
    return 0;
}