//特徴量抽出サンプル
#include <stdio.h>
#include <iostream>
#include <unistd.h>
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

  Mat frame[2];
  Mat orig;

  cap >> orig;
  resize(orig,frame[0],Size(),0.5,0.5);

  for(;;){
    cap >> orig;
    Mat dst;
    resize(orig,dst,Size(),0.5,0.5);
    imshow("Keypoints", dst);
    if(waitKey(1)==13)break;
  }

  cap >> orig;
  resize(orig,frame[1],Size(),0.5,0.5);

  cap.release();

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
  Ptr<SURF> detector = SURF::create();
  // 特徴点情報を格納するための変数
  vector<KeyPoint> keypoints1,keypoints2;
  Mat descriptors1,descriptors2;
  Mat img_keypoints1,img_keypoints2;
  // 特徴点抽出の実行
  //detector->detect(frame, keypoints);
  detector->detectAndCompute(frame[0],noArray(), keypoints1, descriptors1);
  detector->detectAndCompute(frame[1],noArray(), keypoints2, descriptors2);
  //cout<< descriptors <<endl<<endl;

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


  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
  vector< vector<DMatch> > matches;
  matcher->knnMatch(descriptors1, descriptors2, matches, 2); //上位2位までの点を探す
  //descriptor-A: query画像
  ////descriptor-B: train画像

  cout << "size="       << matches.size()         << endl;   

//  cout << "match.size=" << matches[1].size()      << endl;
//  cout << "queryIdx="   << matches[1][0].queryIdx << endl;
//  cout << "trainIdx="   << matches[1][0].trainIdx << endl;
//  cout << "imgIdx="     << matches[1][0].imgIdx   << endl;
//  cout << "distance="   << matches[1][0].distance << endl << endl;

//  cout << "match.size=" << matches[1].size()      << endl;
//  cout << "queryIdx="   << matches[1][1].queryIdx << endl;
//  cout << "trainIdx="   << matches[1][1].trainIdx << endl;
//  cout << "imgIdx="     << matches[1][1].imgIdx   << endl;
//  cout << "distance="   << matches[1][1].distance << endl;

  vector<DMatch> bestMatches;
  vector<KeyPoint> bestkey1,bestkey2;
  Mat outImg;

  float match_par = .5f; //対応点のしきい値
  for (int i=0; i<matches.size(); i++){
    float dist1 = matches[i][0].distance;
    float dist2 = matches[i][1].distance;
    //良い点を残す（最も類似する点と次に類似する点の類似度から）
    if (dist1 <= dist2 * match_par && dist1<=0.2f) {
      //cout << dist1 <<",";
      bestMatches.push_back(matches[i][0]);
      bestkey1.push_back(keypoints1[matches[i][0].queryIdx]);
      bestkey2.push_back(keypoints2[matches[i][0].trainIdx]);
    }
  }
  cout<<endl;
  cout << "BestMatch=" << bestkey1.size() << endl;


  //drawKeypoints( frame[0], bestkey1, img_keypoints1, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  //drawKeypoints( frame[1], bestkey2, img_keypoints2, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  drawMatches( frame[0], keypoints1, frame[1], keypoints2, bestMatches, outImg, Scalar::all(-1), Scalar(127,127,127,255), vector<char>(), DrawMatchesFlags::DEFAULT);

  //-- Show detected (drawn) keypoints
  //imshow("Keypoints1", img_keypoints1 );
  //imshow("Keypoints2", img_keypoints2 );
  imshow("Matches",outImg);

  for(;;){
    int key = waitKey(1);
    if(key == 113)//qボタンが押されたとき
    {
      break;
    }
  }


  return 0;
}
