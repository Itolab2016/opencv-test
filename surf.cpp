#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/xfeatures2d.hpp>

int main()
{
  cv::VideoCapture cap(0);//デバイスのオープン
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
	

  if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
  {
    //読み込みに失敗したときの処理
    return -1;
  }
  cv::xfeatures2d::initModule_xfeatures2d();
  cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("SURF");
  cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create("SURF");


  while(1)//無限ループ
  {
    cv::Mat frame;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;



    cap >> frame; // get a new frame from camera

    //
    //取得したフレーム画像に対して，クレースケール変換や2値化などの処理を書き込む．
    //

    detector->detect(frame, keypoints);
    extractor->compute(frame, keypoints, descriptor);
    cv::Mat output;
    cv::drawKeypoints(frame, keypoints, output);



    cv::imshow("window", output);//画像を表示．

    int key = cv::waitKey(1);
    if(key == 113)//qボタンが押されたとき
    {
      break;//whileループから抜ける．
    }
    else if(key == 115)//sが押されたとき
    {
      //フレーム画像を保存する．
      cv::imwrite("img.png", frame);
    }
  }
  cv::destroyAllWindows();
  return 0;
}
