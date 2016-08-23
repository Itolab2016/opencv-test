#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, const char* argv[])
{
	char str[10];
	const int width=1280;
	const int height=720;
	Mat img1(height, width, CV_8UC1);
	Mat img2(height, width, CV_8UC1);

	namedWindow("image1", cv::WINDOW_NORMAL);
	namedWindow("image2", cv::WINDOW_NORMAL);

	for (int i=0; i<100; i++){
		  
		sprintf(str,"img/img%04d.png",i);
		// 画像データをファイルを読み込む
		img1 = imread(str, cv::IMREAD_GRAYSCALE);
		// 画像の読み込みに失敗したらエラー終了する
		if(img1.empty())
		{
			cerr << "Failed to open image file." << endl;
			return -1; 
		}
		//x方向微分
		for (int y=0; y<height; y++){
			for (int x=0; x<width-5; x++){
				img2.at<unsigned char>(y, x) =img1.at<unsigned char>(y, x+5) - img1.at<unsigned char>(y, x);
				//printf("%d\n",img2.at<unsigned char>(y,x) ); 
			}	
		}
		//img2=img1.clone();
		imshow("image1", img1);
		imshow("image2", img2);
		if(waitKey(300)=='q')break;
	}

	destroyAllWindows();

	return 0;
}
