#include "opencv2/opencv.hpp"


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

  while(1)//無限ループ
  {
    cv::Mat frame;
    cap >> frame; // get a new frame from camera

    //
    //取得したフレーム画像に対して，クレースケール変換や2値化などの処理を書き込む．
    //
    for (int i=0;i<frame.rows;i++){
      for (int j=0;j<frame.cols;j++){
        //frame.data[i*frame.step+j*3+1]=255;
      }
    }

    cv::imshow("window", frame);//画像を表示．

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
