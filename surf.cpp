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
  Mat img_1 = imread("lena_std.tif", IMREAD_GRAYSCALE );
  Mat img_2 = imread("lena_std.tif", IMREAD_GRAYSCALE );
  if( !img_1.data || !img_2.data )
  { 
    std::cout<< " --(!) Error reading images " << std::endl; return -1; 
  }

  // 回転： -40 [deg],  スケーリング： 1.0 [倍]
  float angle = -40.0, scale = 1.0;
  //     // 中心：画像中心
  Point2f center(img_2.cols*0.5, img_2.rows*0.5);
  // 以上の条件から2次元の回転行列を計算
  const Mat affine_matrix = getRotationMatrix2D( center, angle, scale );
  //アフィン変換
  Mat dst_img;
  warpAffine(img_2, dst_img, affine_matrix, img_2.size());



  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 2500;
  Ptr<SURF> detector = SURF::create( minHessian );
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  detector->detect( img_1, keypoints_1 );
  detector->detect( dst_img, keypoints_2 );
  
  //-- Draw keypoints
  Mat img_keypoints_1; Mat img_keypoints_2;
  drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  drawKeypoints( dst_img, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
  //-- Show detected (drawn) keypoints
  imshow("Keypoints 1", img_keypoints_1 );
  imshow("Keypoints 2", img_keypoints_2 );
  
  while(1){
    int key = waitKey(1);
    if(key == 113)//qボタンが押されたとき
    {
      return 0;
    }
  }
  return 0;
}
