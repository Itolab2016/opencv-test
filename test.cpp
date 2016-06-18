#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(int argc, const char* argv[])
{
  // 画像データをファイル（この例では「lena.jpg」）から読み込む
  cv::Mat src = cv::imread("lena.jpg", cv::IMREAD_COLOR);
	printf("%d,%d,%d\n",src.cols,src.rows,src.step);
	printf("%p\n",src.data);
	printf("%p,%p,%d\n",src.datastart,src.dataend,src.dataend-src.datastart);
	printf("%d\n",src.data[0]);
	short i,j;
	for (i=0;i<src.rows;i++){
		for (j=0;j<src.cols;j++){
			src.data[i*src.step+j*3+1]=0;
		}
	}


  // 画像の読み込みに失敗したらエラー終了する
  if(src.empty())
  {
    std::cerr << "Failed to open image file." << std::endl;
    return -1; 
  }

  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  cv::imshow("image", src);
  cv::waitKey(0);
  cv::destroyAllWindows();

  return 0;
}
