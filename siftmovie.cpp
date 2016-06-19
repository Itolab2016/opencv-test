#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

/* @function main */
int main()
{
  VideoCapture cap(0);
  if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
  {
    //読み込みに失敗したときの処理
    return -1;
  }

  while(1){
    Mat orig,frame;
    cap >> orig;
    resize(orig,frame,Size(),0.7,0.7);

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 4000;
    Ptr<SIFT> detector = SIFT::create(minHessian);
    std::vector<KeyPoint> keypoints;
    detector->detect( frame, keypoints );

    //-- Draw keypoints
    Mat img_keypoints;
    drawKeypoints( frame, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

    //-- Show detected (drawn) keypoints
    imshow("Keypoints", img_keypoints );


    int key = waitKey(1);
    if(key == 113)//qボタンが押されたとき
    {
      break;
    }

  }
  return 0;
}
