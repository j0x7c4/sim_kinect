/*
Sample code for using SimKinect
*/
#include "SimpleKinectReader.h" 
#include <iostream>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>

using namespace std;
using namespace cv;

//edit this path if you want to read data from oni file
const char* file_path = "data/drink.oni";
//edit this path if you want to save oni file
char* save_path = "drink.oni";
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
	VideoWriter color_writer;
	VideoWriter depth_writer;
	//Create a SimKinect instance
	SimKinect sensor(video_size_width,video_size_height);
	if ( ret = sensor.Init(file_path) ) {
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
	//Craete data buffer
	color_data = new unsigned char[video_size_width*video_size_height*3];
	depth_data = new unsigned char[video_size_width*video_size_height*3];
	depth_map = new int[video_size_width*video_size_height];
	int frame_cnt =0 ;
	//color_writer.open("rgb.avi",CV_FOURCC('D','I','V','X'),30,cvSize(video_size_width,video_size_height),true);
	//depth_writer.open("dep.avi",CV_FOURCC('D','I','V','X'),30,cvSize(video_size_width,video_size_height),true);
	while ( !quit ) {
		vector<SKUser> users;
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
		//If you need get user information ( joint position ), add argment users
		sensor.GetNextFrame(color_data,depth_data,depth_map,users);
		
		//Draw the frames with OpenCV
		Mat color_img(video_size_height,video_size_width,CV_MAKETYPE(8,3),color_data);
		Mat depth_img(video_size_height,video_size_width,CV_MAKETYPE(8,3),depth_data);
		
		for ( int i=0 ; i<users.size() ; i++ ) {
			DrawUser(users[i],depth_img);  //Add user information ( user label, and joint dot )
			//DrawUser(users[i],color_img);
		}
		//if ( frame_cnt % 10 == 0 ) { 
		//	sprintf(buffer,"drink_rgb_%d.jpg",frame_cnt);
		//	imwrite(buffer,color_img);
		//}
		t = getTickFrequency()/((double)getTickCount()-t);
		sprintf(buffer,"%d",(int)t);
		putText(color_img,string(buffer),cvPoint(10,50),CV_FONT_HERSHEY_SIMPLEX,1,CV_RGB(0,0,0),2);
		putText(depth_img,string(buffer),cvPoint(10,50),CV_FONT_HERSHEY_SIMPLEX,1,CV_RGB(255,255,255),2);
		imshow("COLOR",color_img);
		imshow("DEPTH",depth_img);
		//color_writer<<color_img;
		//depth_writer<<depth_img;
		char key = waitKey(10);
		switch (key) {
		case 27: 
			quit = true;
			break;
		}
		frame_cnt++;
	}
	
	//Stop record before exit
	//sensor.StopRecord();
}