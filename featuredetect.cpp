//特徴量抽出サンプル
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

    // FeatureDetectorオブジェクトの生成
    // 特徴量抽出に使えるのはいかのとおり
    // GFTTDetector
    // AgastFeatureDetector
    // FastFeatureDetector
    // MSER
    // BRISK
    // KAZE
    // ORB
    // AKAZE
    //
    //cv::xfeatures2d::StarDetector OK
    //cv::xfeatures2d::MSDDetector OK
    //cv::xfeatures2d::LATCH  LATCH だめ
    //cv::xfeatures2d::LUCID  LUCID だめ
    //cv::xfeatures2d::BriefDescriptorExtractor だめ
    //cv::xfeatures2d::DAISY だめ
    //cv::xfeatures2d::FREAK だめ
    //cv::xfeatures2d::SIFT OK
    //cv::xfeatures2d::SURF OK
    //
    //
    //
    //
    Ptr<ORB> detector = ORB::create();
    // 特徴点情報を格納するための変数
    vector<KeyPoint> keypoints;
    Mat descriptors;
    Mat img_keypoints;
    // 特徴点抽出の実行
    //detector->detect(frame, keypoints);
    detector->detectAndCompute(frame,noArray(), keypoints, descriptors);
    //cout<< descriptors <<endl<<endl;
    drawKeypoints( frame, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

    //特徴量マッチング
    //タイプ  手法
    //BruteForce  L2ノルム・全探索
    //BruteForce-L1 L1ノルム・全探索
    //BruteForce-Hamming  ハミング距離・全探索
    //BruteForce-Hamming(2) ハミング距離・全探索
    //FlannBased  flann・最近傍探索
    //
    //メソッド  手法
    //match 最も良い点を探す
    //knnMatch  上位k個の良い点を探す
    //radiusMatch 特徴量記述の空間で距離がしきい値以下の点を探す


    //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    //vector<vector<DMatch>> mathes;
    //matcher->knnMatch(descriptor-A, descriptor-B, matches, 2); //上位2位までの点を探す
    //descriptor-A: query画像
    ////descriptor-B: train画像

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
