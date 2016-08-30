//三次元復元研究用プログラム
//乱数で人工的に点群を作成し，平面に透視投影し
//２枚の透視投影画像から回転量と移動量を求める（２０１６．８．３０）
//
//２枚の画像から３次元点を復元する
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

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
  x=((rand()-harf)/harf)*100.0;
  y=((rand()-harf)/harf)*200.0;
  z=300.0+((double)rand()/RAND_MAX)*500.0;
}

int main()
{

  // Example. Estimation of fundamental matrix using the RANSAC algorithm
  int point_count = 1000;

  char file2dv1[20]="test2dv1.dat";
  char file2dv2[20]="test2dv2.dat";
  char file2dv3[20]="test2dv3.dat";
  char file3d[20]  ="test3d.dat";

  my3dpoint p1[point_count];
  vector<Point2f> points1;
  vector<Point2f> points2;
  vector<Point2f> points3;
  vector<Point3f> point3d1(point_count);
  vector<Point3f> point3d2(point_count);
  float fx=300.0;
  float fy=300.0;
  float cx=0.0;
  float cy=0.0;
  Mat K = (Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  //float tx=1,ty=0,tz=0;
  //Mat R1 = (Mat_<float>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 );
  //Mat R2 = (Mat_<float>(3,4) << 1, 0, 0, tx, 0, 1, 0, ty, 0, 0, 1, tz, 0, 0, 0, 1 );

  for(int i=0; i<point_count; i++){
    point3d1[i]=Point3f(p1[i].getX(),p1[i].getY(),p1[i].getZ());
  }
  //cout << point3d1 << endl;
  
  
  float rx=0.0,ry=0.0,rz=3.14159*20/180;
  Mat rvec1=(Mat_<float>(3,1)<<0,0,0);
  Mat rvec2=(Mat_<float>(3,1)<<rx,ry,rz);

  float tx=0,ty=0,tz=0;
  float scale=sqrt(tx*tx+ty*ty+tz*tz);
  Mat tvec1=(Mat_<float>(3,1)<<0,0,0);
  Mat tvec2=(Mat_<float>(3,1)<<tx,ty,tz);
  
  Mat distcoef;
    
  //透視投影実験
  projectPoints(point3d1,rvec1,tvec1,K,distcoef,points1);
  projectPoints(point3d1,rvec2,tvec2,K,distcoef,points2);

  //エッセンシャル行列,Rとt を求める
  Mat E=findEssentialMat(points1,points2,300.0,Point2d(0,0),RANSAC,0.999,1.0);
  Mat R,t;

  recoverPose(E,points1,points2,R,t,300.0,Point2d(0,0));

  //求めたRtを用いて再投影
  projectPoints(point3d1,R,scale*t,K,distcoef,points3);

  //３次元復元
  //triangulatePoints(InputArray projMatr1, InputArray projMatr2, InputArray projPoints1, InputArray projPoints2, OutputArray points4D);

  //投影行列の作成
  Mat M1(3,4,CV_32F);
  Mat M2(3,4,CV_32F);
  Mat T1(3,1,CV_32F);
  Mat T2(3,1,CV_32F);

  M1.at<float>(0,0)=731;

  cout <<"M1=" << M1 << endl;

  //ファイルに記録
  ofstream fp2dv1(file2dv1);
  ofstream fp2dv2(file2dv2);
  ofstream fp2dv3(file2dv3);
  ofstream fp3d(file3d);

  for (int i=0; i<point_count;i++){
    fp3d<<point3d1[i].x<<" "<<point3d1[i].y<<" "<<point3d1[i].z<<endl;
    fp2dv1<<points1[i].x<<" "<<points1[i].y<<endl;
    fp2dv2<<points2[i].x<<" "<<points2[i].y<<endl;
    fp2dv3<<points3[i].x<<" "<<points3[i].y<<endl;
  }  



  Mat R2;
  Rodrigues(rvec2,R2);  
  cout << "R=" << R <<endl<<endl;
  cout << "R2=" << R2 <<endl<<endl;

  //cout << "rvec=" << rvec << endl <<endl;
  //printf("Rvec=[%8.3f,%8.3f,%8.3f]\n",rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0));
  //cout << "t=" << t << endl;
  printf("tvec=[%4.0f,%4.0f,%4.0f]\n",scale*t.at<double>(0,0),scale*t.at<double>(1,0),scale*t.at<double>(2,0));
  

//  cout << R1 << endl;
//  cout << K*R2 << endl;
//  for (int i=0;i<100;i++){
//    p1[i].dispnobr();
//  }

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
