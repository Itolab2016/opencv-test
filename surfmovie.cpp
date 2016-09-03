#特徴量抽出サンプル
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
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

  for(;;/*int i=0; i<1;i++*/){
    Mat orig,frame;
    cap >> orig;
    resize(orig,frame,Size(),0.7,0.7);

// SIFTまたはSURFを使う場合はこれを呼び出す．
// //cv::initModule_nonfree(); 
// FeatureDetectorオブジェクトの生成
  Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();
// 特徴点情報を格納するための変数
  vector<KeyPoint> keypoints;
  Mat img_keypoints;
// 特徴点抽出の実行
  detector->detect(frame, keypoints);
  drawKeypoints( frame, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );


#if 0    
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 4000;
    Ptr<SURF> detector = SURF::create( minHessian );

    std::vector<KeyPoint> keypoints;
    Mat mask,discripter;

    detector->detectAndCompute( frame, mask, keypoints, discripter, false );

    cout << discripter << endl;

    //-- Draw keypoints
    Mat img_keypoints;
    drawKeypoints( frame, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
#endif
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
