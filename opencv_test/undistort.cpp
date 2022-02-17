#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;
using namespace cv;

int main()
{
    Mat src,dst,map1,map2;
    src=imread("/home/gp/桌面/gp_slam/opencv_test/two_image_pose_estimation/1.png");

    Mat cameraMatrix=Mat::eye(3,3,CV_64F);
    cameraMatrix.at<double>(0, 0) = 458.654;//fx
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 367.215;//cx
    cameraMatrix.at<double>(1, 1) = 457.296;//fy
    cameraMatrix.at<double>(1, 2) = 248.375;//cy

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = -0.28340811;//k1
    distCoeffs.at<double>(1, 0) = 0.07395907;//k2
    distCoeffs.at<double>(2, 0) = 0.00019359;//p1
    distCoeffs.at<double>(3, 0) = 1.76187114e-05;//p2

    
    //std::vector<cv::Point2f> src_pts{ cv::Point2f(500,500)};
    //std::vector<cv::Point2f> dst_pts;
    //undistortPoints(src_pts,dst_pts,cameraMatrix,distCoeffs);

    //undistort(src,dst,cameraMatrix,distCoeffs);
    
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, src.size(), 1, src.size(), 0), 
        src.size(), CV_32FC1, map1, map2);
    
    remap(src, dst, map1, map2, INTER_LINEAR);
    cout<<map2.size()<<endl;
    //imshow("输入",map2);
    //imshow("输入",src);
    //imshow("矫正后",dst);

    waitKey(0);
    return 0;
}
