#include "opencv2/opencv.hpp"
#include <stdio.h>

int count=0;

void my_mouse_callback(int event, int x, int y, int flags, void* param){

  cv::Mat* image = static_cast<cv::Mat*>(param);
  char str[20];
  switch (event){
  case cv::EVENT_MOUSEMOVE:
    break;

  case cv::EVENT_LBUTTONDOWN:

    //フレーム画像を保存する．
    sprintf(str,"img/img%04d.png",count++);
    cv::imwrite(str, *image);
    std::cout << str <<std::endl;
    break;

  case cv::EVENT_LBUTTONUP:
    break;
  }
}

int main()
{
  cv::VideoCapture cap(0);//デバイスのオープン
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);

  cv::Mat frame;

  if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
  {
    //読み込みに失敗したときの処理
    return -1;
  }
  cv::namedWindow("hoge", CV_WINDOW_AUTOSIZE);
  // コールバックを設定
  cv::setMouseCallback("hoge", my_mouse_callback, (void *)&frame);

  while(1)//無限ループ
  {
    cap >> frame; // get a new frame from camera

    //
    //取得したフレーム画像に対して，クレースケール変換や2値化などの処理を書き込む．
    //
    //for (int i=0;i<frame.rows;i++){
    //  for (int j=0;j<frame.cols;j++){
        //frame.data[i*frame.step+j*3+1]=255;
    //  }
    //}

    cv::imshow("hoge", frame);//画像を表示．
    

    int key = cv::waitKey(1);
    if(key == 113)//qボタンが押されたとき
    {
      break;//whileループから抜ける．
    }
    else if(key == 13)//Enterが押されたとき
    {
      //フレーム画像を保存する．
      //sprintf(str,"img/img%04d.png",count++);
      //cv::imwrite(str, frame);
    }
  }
  cv::destroyAllWindows();
  return 0;
}
