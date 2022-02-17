#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;

int main(int argc,char *argv[])
{
    //加载图像
    Mat img,gray_image,dst;
    img =cv::imread("/home/gp/桌面/gp_slam/opencv_test/1.png");
    //判断导入成功
    if(img.empty())
    {
        cout<<"加载失败"<<endl;
        return -1;
    }
    //显示图像
    cv::namedWindow("original image",WINDOW_AUTOSIZE);
    cv::imshow("original image",img);
    
    //转换灰度图
    cvtColor(img, gray_image, COLOR_BGR2GRAY);
    int width = gray_image.cols;
	int height = gray_image.rows;
    
    //单通道遍历像素
    ///**
    for(int row=0;row<height;row++)
    {
        for(int col=0;col<width;col++)
        {
            int gray=gray_image.at<uchar>(row,col);
            gray_image.at<uchar>(row,col)=255-gray;
        }
    }
    namedWindow("inv_gray image",WINDOW_AUTOSIZE);
    imshow("inv_gray image",gray_image);
    //**/

    //多通道
    
    dst.create(img.size(), img.type());
	int widths = img.cols;
	int heights = img.rows;
	int td = img.channels();
	for (int row = 0; row < heights; row++)
	{
		for (int col = 0; col < widths; col++)
		{
			int B = img.at<Vec3b>(row, col)[0];
			int G = img.at<Vec3b>(row, col)[1];
			int R = img.at<Vec3b>(row, col)[2];
			dst.at<Vec3b>(row, col)[0] = 255 - B;
			dst.at<Vec3b>(row, col)[1] = 255 - G;
			dst.at<Vec3b>(row, col)[2] = 255 - R;
		}

	}
	namedWindow("inv_rgb_image", WINDOW_AUTOSIZE);
	imshow("inv_rgb_image", dst);

   
    imwrite("./inv_gray.jpg", gray_image);
    waitKey(0);
    return 0;

}


