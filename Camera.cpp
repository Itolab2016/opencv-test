#include<stdio.h>
#include<highgui.h>

int main(int argc,char** argv){
	int key;
	CvCapture *capture;
	IplImage *frameImage;
	char windowNameKamera[]="Capture";


	if((capture=cvCreateCameraCapture(-1))==NULL){
	printf("カメラが見つかりません\n");
	return -1;
	}

	cvNamedWindow(windowNameKamera,CV_WINDOW_AUTOSIZE);

	while(1){

		frameImage=cvQueryFrame(capture);
	
		cvShowImage(windowNameKamera,frameImage);
	
		key=cvWaitKey(1);
		if(key=='q'){
			break;
		}
	}

	cvReleaseCapture(&capture);
	
	cvDestroyWindow(windowNameKamera);

	return 0;

}
