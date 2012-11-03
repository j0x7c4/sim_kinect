/*
Sample code for using SimKinect
*/
#include "SimpleKinectReader.h" 
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

//edit this path if you want to read data from oni file
const char* file_path = "D:/2_side/SideBin01.oni";
//edit this path if you want to save oni file
const char* save_path = "sample.oni";
//default width and height
int video_size_width = 640;
int video_size_height = 480;

int main ( ) {
	unsigned char* depth_data=NULL; 
	unsigned char* color_data=NULL;
	int* depth_map=NULL;
	int ret;
	bool quit = false;
	char buffer[100];
	//Create a SimKinect instance
	SimKinect sensor;
	if ( ret = sensor.Init( ) ) {
		printf("Failed to initialize: %d\n",ret);
		exit(1);
	}
	// If you want to read data from oni file, just use this code block
	//if ( ret = sensor.Init( file_path ) ) {
	//	printf("Failed to initialize: %d\n",ret);
	//	exit(1);
	//}
	//Record the data, if you want to record 
	//sensor.StartRecord(save_path);
	//SimKinect sensor(file_path);
	//Craete data buffer
	color_data = new unsigned char[video_size_width*video_size_height*3];
	depth_data = new unsigned char[video_size_width*video_size_height*3];
	depth_map = new int[video_size_width*video_size_height];
	int frame_cnt =0 ;
	while ( !quit ) {
		double t = (double)getTickCount(); //for calc FPS
		//Get next frame
		//If you only need depth information
		// depth_data: the data for showing depth image
		// depth_map : the data for real depth value
		//sensor.GetNextDepthFrame(depth_data,depth_map);
		//If you only need colr information
		// color_data: the data for showing color image
		//sensor.GetNextColorFrame(color_data);
		//Get both color information and depth information
		sensor.GetNextFrame(color_data,depth_data,depth_map);
		//If you need get user information ( joint position )
		vector<SKUser> users;
		sensor.GetUsers(users);
		//Draw the frames with OpenCV
		Mat color_img(video_size_height,video_size_width,CV_MAKETYPE(8,3),color_data);
		Mat depth_img(video_size_height,video_size_width,CV_MAKETYPE(8,3),depth_data);
		
		for ( int i=0 ; i<users.size() ; i++ ) {
			DrawUser(users[i],depth_img);  //Add user information ( user label, and joint dot )
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