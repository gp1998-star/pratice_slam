
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;
using namespace std;

//-------------------------------------------------------------------------------  
//  CV&AR:Harris
//	Auther: 1210
//	Date:2019/03/21
//-------------------------------------------------------------------------------  

Mat g_srcImage, g_srcImage1, g_grayImage;
//Mat g_srcImage2, g_srcImage3, g_grayImage2;

int thresh = 50; //当前阈值
int max_thresh = 155; //最大阈值

void on_CornerHarris(int, void*);//回调函数

int main()
{
	//载入原始图并进行克隆保存
	g_srcImage = imread("/home/gp/桌面/gp_slam/opencv_test/two_image_pose_estimation/2.png", 1);
	if (!g_srcImage.data) { printf("picture error！\n"); return false; }
	imshow("src 1210", g_srcImage);
	g_srcImage1 = g_srcImage.clone();
	//转为灰度图
	cvtColor(g_srcImage1, g_grayImage, COLOR_BGR2GRAY);
	//调用函数，进行初始化
	on_CornerHarris(0, 0);
	imshow("dst 1210", g_srcImage1);
	waitKey(0);
	return(0);
}

void on_CornerHarris(int, void*)
{
	//定义局部变量
	Mat dstImage;//目标图
	Mat normImage;//归一化后的图
	Mat scaledImage;//线性变换后的八位无符号整型的图

	//清除上一次调用此函数时他们的值
	dstImage = Mat::zeros(g_srcImage.size(), CV_32FC1);
	g_srcImage1 = g_srcImage.clone();

	//进行角点检测
	cornerHarris(g_grayImage, dstImage, 2, 3, 0.04, BORDER_DEFAULT);
	
	// 归一化与转换
	normalize(dstImage, normImage, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(normImage, scaledImage);//将归一化后的图线性变换成8位无符号整型 
	
	// 将检测到的，且符合阈值条件的角点绘制出来
	int num=0;
	for (int j = 0; j < normImage.rows; j++)
	{
		for (int i = 0; i < normImage.cols; i++)
		{
			if ((int)normImage.at<float>(j, i) > thresh + 80)
			{
				circle(g_srcImage1, Point(i, j), 5, Scalar(10, 10, 255), 2, 8, 0);
				circle(scaledImage, Point(i, j), 5, Scalar(0, 10, 255), 2, 8, 0);
				num++;
			}
		}
	}
	imshow("dst 1210", g_srcImage1);
	imwrite("./harris_img.jpg",g_srcImage1);
	cout<<"numbers of point	"<<num<<endl;
}


