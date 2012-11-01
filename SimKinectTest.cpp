#include "SimpleKinectReader.h" 
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

#define file_path "D:/2_side/SideBin01.oni"
void drawDepthImage ( int* depth_map, unsigned char* depth_img );


int main ( ) {
	unsigned char* depth_data=NULL;
	unsigned char* color_data=NULL;
	int* depth_map=NULL;
	int ret;
	bool quit = false;
	char buffer[100];
	//Create a SimKinect instance
	SimKinect sensor;
	if ( ret = sensor.Init(file_path ) ) {
		printf("Failed to initialize: %d\n",ret);
		exit(1);
	}
	//Record the data
	//sensor.StartRecord("sample.oni");
	//SimKinect sensor(file_path);
	//Craete data buffer
	color_data = new unsigned char[640*480*3];
	depth_data = new unsigned char[640*480*3];
	depth_map = new int[640*480];
	int frame_cnt =0 ;
	while ( !quit ) {
		double t = (double)getTickCount();
		//Get next frame
		vector<SKUser> users;
		sensor.GetNextFrame(color_data,depth_data,depth_map);
		sensor.GetUsers(users);
		//cout<<users.size()<<endl;
		//Draw the frame with OpenCV
		Mat color_img(480,640,CV_MAKETYPE(8,3),color_data);
		Mat depth_img(480,640,CV_MAKETYPE(8,3),depth_data);
		
		for ( int i=0 ; i<users.size() ; i++ ) {
			DrawUser(users[i],depth_img);
			DrawUser(users[i],color_img);
		}
		t = getTickFrequency()/((double)getTickCount()-t);
		sprintf(buffer,"%d",(int)t);
		putText(color_img,string(buffer),cvPoint(10,50),CV_FONT_HERSHEY_SIMPLEX,1,CV_RGB(0,0,0),2);
		putText(depth_img,string(buffer),cvPoint(10,50),CV_FONT_HERSHEY_SIMPLEX,1,CV_RGB(255,255,255),2);
		imshow("COLOR",color_img);
		imshow("DEPTH",depth_img);

		char key = waitKey(30);
		switch (key) {
		case 27: 
			quit = true;
			break;
		}

	}
	//Stop record before exit
	//sensor.StopRecord();
}