//三次元復元研究用プログラム
//廊下画像　img/img????.pngを三次元復元するプロトタイププログラム（２０１６．９．６製作開始！）
//
//copyright Kouhei Ito @ Kanazawa ,Japan
//
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
  int point_count = 10;//生成するランダムな３D点の数

  char file3d[20]  ="test3d.dat";//三次元ランダム点座標ファイル
  char file2dv1[20]="test2dv1.dat";//投影１の点座標ファイル
  char file2dv2[20]="test2dv2.dat";//投影２の点座標ファイル
  char file2dv3[20]="test2dv3.dat";//復元した外部パラメータにより再投影した点座標ファイル
  char file3dres[20]="test3dres.dat";//三次元復元結果座標ファイル

  my3dpoint p1[point_count];
  vector<Point2f> points1;
  vector<Point2f> points2;
  vector<Point2f> points3;
  vector<Point3f> point3d1(point_count);
//  vector<Point3f> point3d2(point_count);
  Mat points4d;
  Mat points3d;

  //3D点ベクトル設定
  for(int i=0; i<point_count; i++){
    point3d1[i]=Point3f(p1[i].getX(),p1[i].getY(),p1[i].getZ());
  }
  //cout << point3d1 << endl;

  //内部パラメータ行列作成
  float fx=300.0;
  float fy=300.0;
  float focal = 300.0;
  float cx=0.0;
  float cy=0.0;
  Mat K = (Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  
  //外部パラメータ行列１作成  
  Mat rvec1=(Mat_<float>(3,1)<<0,0,0);
  Mat tvec1=(Mat_<float>(3,1)<<0,0,0);

  //外部パラメータ行列２作成
  float rx=0.0,ry=20.0*3.14159/180,rz=5.0*3.14159/180;
  float tx=1000,ty=0,tz=0;
  float scale=sqrt(tx*tx+ty*ty+tz*tz);
  Mat rvec2=(Mat_<float>(3,1)<<rx,ry,rz);
  Mat tvec2=(Mat_<float>(3,1)<<tx,ty,tz);
  
 
  //改造のポイント
  //points1,points2の実画像の特徴点のキーポイントを入れれば良い
  int frn=0;
  char str[50];
  Mat orig;
  Mat frame[2];

    cout << "ImgNo=" << frn << "," << frn+1 << endl;
    sprintf(str,"img/img%04d.png",frn);
    orig = imread(str, IMREAD_COLOR);
    resize(orig,frame[0],Size(),0.5,0.5);

    sprintf(str,"img/img%04d.png",frn+1);
    orig = imread(str, IMREAD_COLOR);
    resize(orig,frame[1],Size(),0.5,0.5);
 
    Ptr<SURF> detector = SURF::create();
    // 特徴点情報を格納するための変数
    vector<KeyPoint> keypoints1,keypoints2;
    Mat descriptors1,descriptors2;
    Mat img_keypoints1,img_keypoints2;
    // 特徴点抽出の実行
    detector->detectAndCompute(frame[0],noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(frame[1],noArray(), keypoints2, descriptors2);
    
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
    cout << "BestMatch=" << bestkey1.size() << endl;
    cout<<endl;


    //drawKeypoints( frame[0], bestkey1, img_keypoints1, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    //drawKeypoints( frame[1], bestkey2, img_keypoints2, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    drawMatches( frame[0], keypoints1, frame[1], keypoints2, bestMatches, outImg, Scalar::all(-1), Scalar(127,127,127,255), vector<char>(), DrawMatchesFlags::DEFAULT);


  //エッセンシャル行列,Rとt を求める
  Mat R,t,mask,distcoef;
  Mat E=findEssentialMat(bestkey1,bestkey2,focal,Point2d(cx,cy),RANSAC,0.999,1.0,mask);
  recoverPose(E,bestkey1,bestkey2,R,t,focal,Point2d(cx,cy),mask);
  t =t*scale;
  projectPoints(point3d1,R,t,K,distcoef,points3);

  //３次元復元
  //triangulatePoints(InputArray projMatr1, InputArray projMatr2, InputArray projPoints1, InputArray projPoints2, OutputArray points4D);

  //投影行列の作成
  Mat M1(3,4,CV_32F);
  Mat M2(3,4,CV_32F);
  Mat R1(3,3,CV_32F);
  //Mat R2(3,3,CV_32F);
  Mat T1(3,1,CV_32F);
  Mat T2(3,1,CV_32F);

  Rodrigues(rvec1,R1);  
  t=scale*t;

  for (int m=0; m<3;m++){
    for (int n=0; n<4; n++){
      if(n==3){
        M1.at<float>(m,n)=tvec1.at<float>(m,0);
        M2.at<float>(m,n)=t.at<double>(m,0);
      }
      else {
        M1.at<float>(m,n)=R1.at<float>(m,n);
        M2.at<float>(m,n)=R.at<double>(m,n);
      }
    }
  }

//  cout <<"M1=" << M1 << endl;
//  cout <<"R1=" << R1 << endl;
//  cout <<"tvec1="<<tvec1<< endl;
//  cout <<"M2=" << M2 << endl;
//  cout <<"R=" << R << endl;
//  cout <<"t="<< t << endl;
  
  M1=K*M1;
  M2=K*M2;
  
  //三角法で３次元復元  
  triangulatePoints(M1, M2, points1, points2, points4d);
  convertPointsFromHomogeneous(points4d.t(),points3d);
  
  //結果出力
//  cout<<points4d.t()<<endl<<endl;
//  cout<<point3d1<<endl<<endl;
//  cout<<points3d<<endl;


  //ファイルに記録
  ofstream fp2dv1(file2dv1);
  ofstream fp2dv2(file2dv2);
  ofstream fp3d(file3d);
  ofstream fp3dres(file3dres);

  for (int i=0; i<point_count;i++){
    fp3d    << point3d1[i].x << " " <<point3d1[i].y << " " << point3d1[i].z << endl;
    fp3dres << points3d.at<float>(i,0) << " " <<points3d.at<float>(i,1) << " " << points3d.at<float>(i,2) << endl;
    fp2dv1<<points1[i].x<<" "<<points1[i].y<<endl;
    fp2dv2<<points2[i].x<<" "<<points2[i].y<<endl;
  }  


}
