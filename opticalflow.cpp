#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, const char* argv[])
{
	char str[10];
	Mat img1,img2;

	namedWindow("image", cv::WINDOW_AUTOSIZE);

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
		for (int y=0; y<300; y++){
			for (int x=0; x<300; x++){
				img2.at<unsigned char>(y, x) =img1.at<unsigned char>(y,x+1)-img1.at<unsigned char>(y, x); 
			}	
		}
		//img2=img1.clone();
		imshow("image", img2);
		if(waitKey(60)=='q')break;
	}

	destroyAllWindows();

	return 0;
}
