#include <stdio.h>
#include <stdlib.h>
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


class my3dpoint{
  double x,y,z;


public:

  my3dpoint();

  double getX(){return x;}
  double getY(){return y;}
  double getZ(){return z;}
  void disp(){cout<<"["<<x<<","<<y<<","<<z<<"]"<<std::endl;}
  void dispnobr(){cout<<x<<" "<<y<<" "<<z<<std::endl;}

};

my3dpoint::my3dpoint(){
  double harf=RAND_MAX/2.0;
  x=((rand()-harf)/harf)*10.0;
  y=((rand()-harf)/harf)*10.0;
  z=((rand()-harf)/harf)*10.0;

}





int main()
{

  my3dpoint p1[100];
  // Example. Estimation of fundamental matrix using the RANSAC algorithm
  int point_count = 100;
  vector<Point2f> points1(point_count);
  vector<Point2f> points2(point_count);
  vector<Point3f> point3d1(point_count);
  vector<Point3f> point3d2(point_count);
  Mat K = (Mat_<float>(3,3) << 1, 0, 320, 0, 1, 240, 0, 0, 1);
  float tx=1,ty=0,tz=0;
  Mat R1 = (Mat_<float>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 );
  Mat R2 = (Mat_<float>(3,4) << 1, 0, 0, tx, 0, 1, 0, ty, 0, 0, 1, tz, 0, 0, 0, 1 );

//  cout << K << endl;
//  cout << R1 << endl;
//  cout << K*R2 << endl;
  for (int i=0;i<100;i++){
    p1[i].dispnobr();
  }

#if 0
  // initialize the points here ... */
  for( int i = 0; i < point_count; i++ )
  {
    points1[i] = ...;
    points2[i] = ...;
  }

  double focal = 1.0;
  cv::Point2d pp(0.0, 0.0);
  Mat E, R, t, mask;

  E = findEssentialMat(points1, points2, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points1, points2, R, t, focal, pp, mask);


///////////////////////////////

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
  std::cout<< keypoints_1[0].pt <<std::endl;


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
#endif 
}
