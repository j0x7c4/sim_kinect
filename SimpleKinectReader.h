/*
  Author: Jie Fu
  Email: china.eli@gmail.com
  This work helps extract the color 
  image and depth image with depth 
  map from RGB-D sensor(kinect, xtion).
  That will be more convenient than original API.
  Now this class only support for OpenNI, however,
  it will support for Microsoft Kinect SDK soon.
*/
#ifndef SIMPLE_KINECT_READER_H
#define SIMPLE_KINECT_READER_H
#include <XnCppWrapper.h>
#include <iostream>
#include <map>
#include <vector>
#include <cv.h>
using namespace std;
using namespace xn;
#endif

#define MAX_DEPTH 10000
#define nColors 10
#define XML_CONFIG "E:/source/hands-tracker/sim_kinect/config.xml"
#define SK_JOINT_NUMBER 25



int CHECK_RC(int nRetVal, char* what);
static UserGenerator user_generator;

enum {
	ERR_OK,
	ERR_USAGE,
	ERR_DEVICE,
	ERR_UNKNOWN
};
typedef enum SKSkeletonJoint
{
	SK_SKEL_COM				= 0,
	SK_SKEL_HEAD			= 1,
	SK_SKEL_NECK			= 2,
	SK_SKEL_TORSO			= 3,
	SK_SKEL_WAIST			= 4,

	SK_SKEL_LEFT_COLLAR		= 5,
	SK_SKEL_LEFT_SHOULDER	= 6,
	SK_SKEL_LEFT_ELBOW		= 7,
	SK_SKEL_LEFT_WRIST		= 8,
	SK_SKEL_LEFT_HAND		= 9,
	SK_SKEL_LEFT_FINGERTIP	=10,

	SK_SKEL_RIGHT_COLLAR	=11,
	SK_SKEL_RIGHT_SHOULDER	=12,
	SK_SKEL_RIGHT_ELBOW		=13,
	SK_SKEL_RIGHT_WRIST		=14,
	SK_SKEL_RIGHT_HAND		=15,
	SK_SKEL_RIGHT_FINGERTIP	=16,

	SK_SKEL_LEFT_HIP		=17,
	SK_SKEL_LEFT_KNEE		=18,
	SK_SKEL_LEFT_ANKLE		=19,
	SK_SKEL_LEFT_FOOT		=20,

	SK_SKEL_RIGHT_HIP		=21,
	SK_SKEL_RIGHT_KNEE		=22,
	SK_SKEL_RIGHT_ANKLE		=23,
	SK_SKEL_RIGHT_FOOT		=24	
} SKSkeletonJoint;

typedef struct simKinect_point3D {
	float x,y,z;
}SKPoint3D;
typedef struct simKinect_point2D {
	int x,y;
}SKPoint2D;

typedef struct simKinect_user{
	int userId;
	SKPoint3D real_joints[25];
	SKPoint2D proj_joints[25];
}SKUser;

static std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;
void DrawUser ( const SKUser& user, cv::Mat& image_data);
class SimKinect {
protected:	
	int video_size_width;
  int video_size_height;
private:
	XnChar* strDepthImageTitle;
	XnChar* strRGBImageTitle;
	//Create a UserGenerator node
	static UserGenerator user_generator;
	//Create a DepthGenerator node
	Device device;
	DepthGenerator depth_generator;
	ImageGenerator color_generator;
	IRGenerator ir_generator;
	AudioGenerator audio_generator;
	ScriptNode script_node;
	// create recorder
	Recorder recorder;
	static XnChar* strPose;


	Context context;
	Player player;
	
	XnBool track_joints[25];
	
	float pDepthHist[MAX_DEPTH];
	char* record_file;

	static XnBool bNeedPose;
	XnBool bDrawUserColor;
	XnBool bDrawBackground;
	XnBool bDrawPixels;
	XnBool bDrawSkeleton;
	XnBool bPrintID;
	XnBool bPrintState;
	XnBool bPause;
	XnBool bRecord;
	XnBool bQuit;
	//device state
	XnBool bIsDepthOn;
	XnBool bIsImageOn;
	XnBool bIsIROn;
	XnBool bIsAudioOn;
	XnBool bIsPlayerOn;
	XnBool bIsUserOn;
	SceneMetaData sceneMD;
  DepthMetaData depthMD;
	ImageMetaData rgbMD;


	unsigned char* color_frame;
	unsigned char* depth_frame;
	int* depth_map;

	static void XN_CALLBACK_TYPE User_NewUser(UserGenerator& generator, XnUserID nId, void* pCookie);
	static void XN_CALLBACK_TYPE User_LostUser(UserGenerator& generator, XnUserID nId, void* pCookie);
	static void XN_CALLBACK_TYPE UserPose_PoseDetected(PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie);
	static void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(SkeletonCapability& capability, XnUserID nId, void* pCookie);
	static void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie);
	void SaveCalibration();
	void LoadCalibration();
	XnStatus OpenDevice(const char* csXmlFile, EnumerationErrors & errors);
	void OpenCommon();
	void LoadArgs ( );
	void GetJoint( XnUserID player, XnSkeletonJoint eJoint , SKPoint3D& rjoint, SKPoint2D& pjoint);
	static void XN_CALLBACK_TYPE MyCalibrationInProgress(SkeletonCapability& capability, XnUserID id, XnCalibrationStatus calibrationError, void* pCookie)
	{
		m_Errors[id].first = calibrationError;
	}
	static void XN_CALLBACK_TYPE MyPoseInProgress(PoseDetectionCapability& capability, const XnChar* strPose, XnUserID id, XnPoseDetectionStatus poseError, void* pCookie)
	{
		m_Errors[id].second = poseError;
	}
	void DrawRGBMap(const ImageMetaData& imd);
	void DrawDepthMapWithUsers(const DepthMetaData& dmd, const SceneMetaData& smd);
	void DrawDepthMap(const DepthMetaData& dmd);
public:
	SimKinect();
	XnStatus Init();
	XnStatus Init(const char* filename);
	XnStatus Uninit();
	void StartRecord(char* save_name);
	void StopRecord();
	void GetNextColorFrame(unsigned char* color_frame);
	void GetNextDepthFrame(unsigned char* depth_frame, int* depth_map);
	void GetNextFrame(unsigned char* color_frame, unsigned char* depth_frame, int* depth_map);
	void GetUsers (vector<SKUser>& users);
	
	~SimKinect();
};

