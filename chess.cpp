#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(){
  Mat img;
  //Mat gray;

  img = imread("chessboard2.jpg");
  //cvtColor(img, gray, COLOR_BGR2GRAY);

  Size boardSize(5, 9);
  vector<Point2f> pointBuf;

 
  bool found = findChessboardCorners( img,
                                 boardSize,
                                 pointBuf,
                                 CV_CALIB_CB_ADAPTIVE_THRESH | 
                                 CV_CALIB_CB_FAST_CHECK /*| 
                                 CV_CALIB_CB_NORMALIZE_IMAGE*/);
  //上記はCV_CALIB_CB_NORMALIZE_IMAGEがコメントアウトされているが、
  //このフラグを有効にした場合は、コーナー検出が失敗する。
  //多くのWebサイトでサンプルにはこのフラグが有効になっているが
  //私の実験ではすべて失敗したので、コメントアウトされている

  cout << "Find? " << found << endl;

  drawChessboardCorners(img, boardSize, Mat(pointBuf), found);
  namedWindow("chessboard", CV_WINDOW_NORMAL);
  imshow("chessboard",img);

  waitKey(0);
  destroyAllWindows();

}
