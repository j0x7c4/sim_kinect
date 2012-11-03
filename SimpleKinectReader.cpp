#include "SimpleKinectReader.h"
#include "cv.h"

XnFloat Colors[][3] =
{
	{0,1,1},
	{0,0,1},
	{0,1,0},
	{1,1,0},
	{1,0,0},
	{1,.5,0},
	{.5,1,0},
	{0,.5,1},
	{.5,0,1},
	{1,1,.5},
	{1,1,1}
};

XnChar* SimKinect::strPose = "Psi";
XnBool SimKinect::bNeedPose = FALSE;
xn::UserGenerator SimKinect::user_generator;
int CHECK_RC(int nRetVal, char* what) {
  if (nRetVal != XN_STATUS_OK) {																
    cout<<what<<" failed: "<<xnGetStatusString(nRetVal)<<endl;
    exit(nRetVal);												
  } 
  else { 
    cout<<what<<" successed!"<<endl; 
    return nRetVal;
  }
}

const XnChar* GetCalibrationErrorString(XnCalibrationStatus error)
{
	switch (error)
	{
	case XN_CALIBRATION_STATUS_OK:
		return "OK";
	case XN_CALIBRATION_STATUS_NO_USER:
		return "NoUser";
	case XN_CALIBRATION_STATUS_ARM:
		return "Arm";
	case XN_CALIBRATION_STATUS_LEG:
		return "Leg";
	case XN_CALIBRATION_STATUS_HEAD:
		return "Head";
	case XN_CALIBRATION_STATUS_TORSO:
		return "Torso";
	case XN_CALIBRATION_STATUS_TOP_FOV:
		return "Top FOV";
	case XN_CALIBRATION_STATUS_SIDE_FOV:
		return "Side FOV";
	case XN_CALIBRATION_STATUS_POSE:
		return "Pose";
	default:
		return "Unknown";
	}
}
const XnChar* GetPoseErrorString(XnPoseDetectionStatus error)
{
	switch (error)
	{
	case XN_POSE_DETECTION_STATUS_OK:
		return "OK";
	case XN_POSE_DETECTION_STATUS_NO_USER:
		return "NoUser";
	case XN_POSE_DETECTION_STATUS_TOP_FOV:
		return "Top FOV";
	case XN_POSE_DETECTION_STATUS_SIDE_FOV:
		return "Side FOV";
	case XN_POSE_DETECTION_STATUS_ERROR:
		return "General error";
	default:
		return "Unknown";
	}
}

// Callback: New user was detected
void XN_CALLBACK_TYPE  SimKinect::User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf("%d New User %d\n", epochTime, nId);
  user_generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE  SimKinect::User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf("%d Lost user %d\n", epochTime, nId);	
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE  SimKinect::UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie) {
  XnUInt32 epochTime = 0;
	
  xnOSGetEpochTime(&epochTime);
  printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
  user_generator.GetPoseDetectionCap().StopPoseDetection(nId);
  user_generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE SimKinect::UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
  XnUInt32 epochTime = 0;
  xnOSGetEpochTime(&epochTime);
  printf("%d Calibration started for user %d\n", epochTime, nId);
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE  SimKinect::UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie) {
  XnUInt32 epochTime = 0;
	
  xnOSGetEpochTime(&epochTime);
  if (eStatus == XN_CALIBRATION_STATUS_OK) {
    // Calibration succeeded
    printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);		
    user_generator.GetSkeletonCap().StartTracking(nId);
  }
  else {
    // Calibration failed
    printf("%d Calibration failed for user %d\n", epochTime, nId);
    user_generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }
}

#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"

// Save calibration to file
void  SimKinect::SaveCalibration() {
  XnUserID aUserIDs[20] = {0};
  XnUInt16 nUsers = 20;
  user_generator.GetUsers(aUserIDs, nUsers);
  for (int i = 0; i < nUsers; ++i) {
    // Find a user who is already calibrated
    if (user_generator.GetSkeletonCap().IsCalibrated(aUserIDs[i])) {
      // Save user's calibration to file
      user_generator.GetSkeletonCap().SaveCalibrationDataToFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
      break;
    }
  }
}
// Load calibration from file
void  SimKinect::LoadCalibration() {
  XnUserID aUserIDs[20] = {0};
  XnUInt16 nUsers = 20;
  user_generator.GetUsers(aUserIDs, nUsers);
  for (int i = 0; i < nUsers; ++i) {
    // Find a user who isn't calibrated or currently in pose
    if (user_generator.GetSkeletonCap().IsCalibrated(aUserIDs[i])) continue;
    if (user_generator.GetSkeletonCap().IsCalibrating(aUserIDs[i])) continue;

    // Load user's calibration from file
    XnStatus rc = user_generator.GetSkeletonCap().LoadCalibrationDataFromFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
    if (rc == XN_STATUS_OK) {
      // Make sure state is coherent
      user_generator.GetPoseDetectionCap().StopPoseDetection(aUserIDs[i]);
      user_generator.GetSkeletonCap().StartTracking(aUserIDs[i]);
    }
    break;
  }
}
void SimKinect::LoadArgs () {

	strDepthImageTitle = "depth";
	strRGBImageTitle = "RGB";
	video_size_width = 640;
	video_size_height = 480;
	bDrawUserColor = FALSE;
	bDrawBackground = TRUE;
	bDrawPixels = TRUE;
	bDrawSkeleton = FALSE;
	bPrintID = FALSE;
	bPrintState = FALSE;
	
	bPause = false;
	bRecord = false;
	bQuit = false;
}
SimKinect::~SimKinect () {
	Uninit();
}
SimKinect::SimKinect ( ) {
	LoadArgs();
	record_file = NULL;
}
SimKinect::SimKinect (char* filename) {
	LoadArgs();
}
XnStatus SimKinect::Uninit() {
	color_generator.Release();
	depth_generator.Release();
  user_generator.Release();
  context.Release();
	if ( bIsDepthOn ) {
		delete[] depth_frame; 
		delete[] depth_map;
	}
	if ( bIsImageOn ) {
		delete[] color_frame;
	}
	return 0;
}
XnStatus SimKinect::Init(char* filename) {
	record_file = new char[255];
	strcpy(record_file,filename);
	xnSetPlayerRepeat(player,FALSE);
	Init();
	return 0;
}
void setErrorState(const char* strMessage)
{
	cout<<strMessage<<endl;
}

void XN_CALLBACK_TYPE onErrorStateChanged(XnStatus errorState, void* /*pCookie*/)
{
	if (errorState != XN_STATUS_OK)
	{
		setErrorState(xnGetStatusString(errorState));
	}
	else
	{
		setErrorState(NULL);
	}
}

void SimKinect::OpenCommon()
{
	XnStatus nRetVal = XN_STATUS_OK;

	bIsDepthOn = false;
	bIsImageOn = false;
	bIsIROn = false;
	bIsAudioOn = false;
	bIsPlayerOn = false;
	bIsUserOn = false; 
	NodeInfoList list;
	nRetVal = context.EnumerateExistingNodes(list);
	if (nRetVal == XN_STATUS_OK)
	{
		for (NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it)
		{
			switch ((*it).GetDescription().Type)
			{
			case XN_NODE_TYPE_DEVICE:
				(*it).GetInstance(device);
				break;
			case XN_NODE_TYPE_DEPTH:
				bIsDepthOn = true;
				(*it).GetInstance(depth_generator);
				break;
			case XN_NODE_TYPE_IMAGE:
				bIsImageOn = true;
				(*it).GetInstance(color_generator);
				break;
			case XN_NODE_TYPE_IR:
				bIsIROn = true;
				(*it).GetInstance(ir_generator);
				break;
			case XN_NODE_TYPE_AUDIO:
				bIsAudioOn = true;
				(*it).GetInstance(audio_generator);
				break;
			case XN_NODE_TYPE_PLAYER:
				bIsPlayerOn = true;
				(*it).GetInstance(player);
				break;
			case XN_NODE_TYPE_USER :
				bIsUserOn = true;
				(*it).GetInstance(user_generator);
				break;
			}
		}
	}

	XnCallbackHandle hDummy;
	context.RegisterToErrorStateChange(onErrorStateChanged, NULL, hDummy);

}
XnStatus SimKinect::OpenDevice(const char* csXmlFile, EnumerationErrors& errors)
{
	XnStatus nRetVal = XN_STATUS_OK;
	if (record_file !=NULL)
	{
		nRetVal = context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = context.OpenFileRecording(record_file,player);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", record_file, xnGetStatusString(nRetVal));
			return nRetVal;
		}
	}
	else {
		nRetVal = context.InitFromXmlFile(csXmlFile, script_node, &errors);
		XN_IS_STATUS_OK(nRetVal);
	}

	OpenCommon();

	return (XN_STATUS_OK);
}

XnStatus SimKinect::Init() {
	XnStatus nRetVal = XN_STATUS_OK;
	xn::EnumerationErrors errors;
	
	nRetVal = OpenDevice(XML_CONFIG, errors);
	
	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (nRetVal);
	}
	else if (nRetVal != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return (nRetVal);
	}
	if ( bIsDepthOn ) {
		depth_frame = new unsigned char[640*480*3];
		depth_map = new int[640*480];
	}
	if ( bIsImageOn ) {
		color_frame = new unsigned char[640*480*3];
	}
	
	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
	if (!user_generator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}
	nRetVal = user_generator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	CHECK_RC(nRetVal, "Register to user callbacks");
	nRetVal = user_generator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
	CHECK_RC(nRetVal, "Register to calibration start");
	nRetVal = user_generator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
	CHECK_RC(nRetVal, "Register to calibration complete");

	if (user_generator.GetSkeletonCap().NeedPoseForCalibration())
	{
		bNeedPose = TRUE;
		if (!user_generator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("Pose required, but not supported\n");
			return 1;
		}
		nRetVal = user_generator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
		CHECK_RC(nRetVal, "Register to Pose Detected");
		user_generator.GetSkeletonCap().GetCalibrationPose(strPose);
	}

	user_generator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = user_generator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
	CHECK_RC(nRetVal, "Register to calibration in progress");

	nRetVal = user_generator.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress);
	CHECK_RC(nRetVal, "Register to pose in progress");

	nRetVal = context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
	return 0;
}

unsigned int getClosestPowerOfTwo(unsigned int n)
{
	unsigned int m = 2;
	while(m < n) m<<=1;

	return m;
}

void SimKinect::DrawRGBMap(const xn::ImageMetaData& imd) {
	static bool bInitialized = false;
	static int width;
	static int height;
	static int width_step;
	static XnRGB24Pixel* g_pTexMap;
	//static Mat color_img(cvSize(640,480),CV_MAKETYPE(8,3));
	if(!bInitialized) {
		width = video_size_width;
		height = video_size_height;
		g_pTexMap = (XnRGB24Pixel*)malloc(width * height * sizeof(XnRGB24Pixel));
		width_step = width*3;
		bInitialized = true;
	}

	const XnUInt8* pImage = imd.Data();
	const XnRGB24Pixel* pImageRow = imd.RGB24Data();
	XnRGB24Pixel* pTexRow = g_pTexMap + imd.YOffset() * width;
	
	for (XnUInt y = 0; y < imd.YRes(); ++y)
	{
		const XnRGB24Pixel* pImage = pImageRow;
		XnRGB24Pixel* pTex = pTexRow + imd.XOffset();

		for (XnUInt x = 0; x < imd.XRes(); ++x, ++pImage, ++pTex)
		{
			*pTex = *pImage;
		}

		pImageRow += imd.XRes();
		pTexRow += width;
	}

	//Draw rgb image
  XnRGB24Pixel* pix_ptr = g_pTexMap;
  
  for ( int i=0 ; i<height ; i++ ) {
		unsigned char* row_pointer = color_frame+i*width_step;
    for ( int j=0 ; j<width_step; j+=3,pix_ptr++ ) {
			row_pointer[j] = pix_ptr->nBlue;
      row_pointer[j+1]=pix_ptr->nGreen;
      row_pointer[j+2]=pix_ptr->nRed;
    }
  }
	//imshow("COLOR",color_img);
	
}
void SimKinect::GetNextFrame(unsigned char* _color_frame, unsigned char* _depth_frame, int* _depth_map) {
	if (!bPause) {
		// Read next available data
		context.WaitAnyUpdateAll();
		if ( color_generator.IsValid() ) {
			color_generator.GetMetaData(rgbMD);
			DrawRGBMap(rgbMD);
			memcpy(_color_frame,color_frame,video_size_height*video_size_width*3*sizeof(unsigned char));
		}
		if ( depth_generator.IsValid() ) {
			depth_generator.GetMetaData(depthMD);
			if ( user_generator.IsValid() ) {
				user_generator.GetUserPixels(0, sceneMD);
				DrawDepthMapWithUsers(depthMD,sceneMD);
			}
			else {
				DrawDepthMap(depthMD);
			}
			memcpy(_depth_frame,depth_frame,video_size_height*video_size_width*3*sizeof(unsigned char));
			memcpy(_depth_map,depth_map,video_size_height*video_size_width*sizeof(int));
		}
	}
}
void SimKinect::GetNextColorFrame(unsigned char* frame) {
	if ( !bPause && color_generator.IsValid() ) {
		// Read next available data
		context.WaitOneUpdateAll(color_generator);
		color_generator.GetMetaData(rgbMD);
		DrawRGBMap(rgbMD);
		memcpy(frame,color_frame,video_size_height*video_size_width*3);
	}
}

void SimKinect::GetNextDepthFrame(unsigned char* frame, int* d_map) {
	if ( !bPause && depth_generator.IsValid() ) {
		// Read next available data
		context.WaitOneUpdateAll(depth_generator);
		depth_generator.GetMetaData(depthMD);
		if (user_generator.IsValid() ) {
			user_generator.GetUserPixels(0, sceneMD);
			DrawDepthMapWithUsers(depthMD, sceneMD);
		}
		DrawDepthMap(depthMD);
		// Process the data
		memcpy(frame,depth_frame,video_size_height*video_size_width*3*sizeof(unsigned char));
		memcpy(d_map,depth_map,video_size_height*video_size_width*sizeof(int));
	}
}
void SimKinect::DrawDepthMap(const xn::DepthMetaData& dmd) {
	static bool bInitialized = false;	
	//image used in opencv;
  
  static unsigned char* pDepthTexBuf;
	static int texWidth, texHeight;

	float topLeftX;
	float topLeftY;
	float bottomRightY;
	float bottomRightX;
	float texXpos;
	float texYpos;

	if(!bInitialized)
	{
		texWidth =  getClosestPowerOfTwo(dmd.XRes());
		texHeight = getClosestPowerOfTwo(dmd.YRes());
    pDepthTexBuf = (unsigned char*)malloc(dmd.XRes()*dmd.YRes()*3*sizeof(unsigned char));
//		printf("Initializing depth texture: width = %d, height = %d\n", texWidth, texHeight);
		bInitialized = true;

		topLeftX = dmd.XRes();
		topLeftY = 0;
		bottomRightY = dmd.YRes();
		bottomRightX = 0;
		texXpos =(float)dmd.XRes()/texWidth;
		texYpos  =(float)dmd.YRes()/texHeight;
	}
  
	unsigned int nValue = 0;
  unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
	XnUInt16 nXRes = dmd.XRes();
	XnUInt16 nYRes = dmd.YRes();

	unsigned char* pDestImage = pDepthTexBuf;

	const XnDepthPixel* pDepth = dmd.Data();
  int* p_depth_map = depth_map;
  //Calculate the accumulative histogram
	memset(pDepthHist, 0, MAX_DEPTH*sizeof(float));
	for (nY=0; nY<nYRes; nY++)
	{
		for (nX=0; nX<nXRes; nX++)
		{
			nValue = *pDepth;
      *p_depth_map++ = nValue;
			if (nValue != 0)
			{
				pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}
  
	for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		pDepthHist[nIndex] += pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}
  pDepth = dmd.Data();
  
	if (bDrawPixels)
	{
		XnUInt32 nIndex = 0;
		// Prepare the texture map
		for (nY=0; nY<nYRes; nY++)
		{
			for (nX=0; nX < nXRes; nX++, nIndex++)
			{

				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;
        
				if (bDrawBackground )
				{
					nValue = *pDepth;

					if (nValue != 0)
					{
            nHistValue = pDepthHist[nValue];

						pDestImage[0] = nHistValue ; 
						pDestImage[1] = nHistValue ;
						pDestImage[2] = nHistValue ;
            
					}
				}

				pDepth++;
				pDestImage+=3;
			}
		}
	}
	else
	{
		xnOSMemSet(pDepthTexBuf, 0, 3*nXRes*nYRes);
	}
  
  //Draw depth image
  unsigned char* pix_ptr = pDepthTexBuf;
  
  for ( int i=0 ; i<video_size_height ; i++ ) {
    unsigned char* row_pointer = depth_frame+i*video_size_width*3;
    for ( int j=0 ; j<video_size_width*3; j+=3 ) {
      row_pointer[j] = *pix_ptr++;
      row_pointer[j+1]=*pix_ptr++;
      row_pointer[j+2]=*pix_ptr++;
    }
  }
}
void SimKinect::DrawDepthMapWithUsers(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd)
{
	static bool bInitialized = false;	
	//image used in opencv;
  
  static unsigned char* pDepthTexBuf;
	static int texWidth, texHeight;

	float topLeftX;
	float topLeftY;
	float bottomRightY;
	float bottomRightX;
	float texXpos;
	float texYpos;

	if(!bInitialized)
	{
		texWidth =  getClosestPowerOfTwo(dmd.XRes());
		texHeight = getClosestPowerOfTwo(dmd.YRes());
    pDepthTexBuf = (unsigned char*)malloc(dmd.XRes()*dmd.YRes()*3*sizeof(unsigned char));
//		printf("Initializing depth texture: width = %d, height = %d\n", texWidth, texHeight);
		bInitialized = true;

		topLeftX = dmd.XRes();
		topLeftY = 0;
		bottomRightY = dmd.YRes();
		bottomRightX = 0;
		texXpos =(float)dmd.XRes()/texWidth;
		texYpos  =(float)dmd.YRes()/texHeight;
	}
  
	unsigned int nValue = 0;
  unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
	XnUInt16 nXRes = dmd.XRes();
	XnUInt16 nYRes = dmd.YRes();

	unsigned char* pDestImage = pDepthTexBuf;

	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pLabels = smd.Data();
  int* p_depth_map = depth_map;
  //Calculate the accumulative histogram
	memset(pDepthHist, 0, MAX_DEPTH*sizeof(float));
	for (nY=0; nY<nYRes; nY++)
	{
		for (nX=0; nX<nXRes; nX++)
		{
			nValue = *pDepth;
      *p_depth_map++ = nValue;
			if (nValue != 0)
			{
				pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}
  
	for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		pDepthHist[nIndex] += pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}
  pDepth = dmd.Data();
  
	if (bDrawPixels)
	{
		
		// Prepare the texture map
		for (nY=0; nY<nYRes; nY++)
		{
			for (nX=0; nX < nXRes; nX++)
			{

				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;
				if (bDrawBackground || *pLabels != 0)
				{
					nValue = *pDepth;
					XnLabel label = *pLabels;
					XnUInt32 nColorID = label % nColors;
					if (label == 0)
					{
						nColorID = nColors;
					}

					if (nValue != 0)
					{
						nHistValue = pDepthHist[nValue];

						pDestImage[0] = nHistValue * Colors[nColorID][0]; 
						pDestImage[1] = nHistValue * Colors[nColorID][1];
						pDestImage[2] = nHistValue * Colors[nColorID][2];
					}
				}

				pDepth++;
				pLabels++;
				pDestImage+=3;
			}
		}
	}
	else
	{
		xnOSMemSet(pDepthTexBuf, 0, 3*nXRes*nYRes);
	}
  
  //Draw depth image
  unsigned char* pix_ptr = pDepthTexBuf;
  
  for ( int i=0 ; i<video_size_height ; i++ ) {
    unsigned char* row_pointer = depth_frame+i*video_size_width*3;
    for ( int j=0 ; j<video_size_width*3; j+=3 ) {
      row_pointer[j] = *pix_ptr++;
      row_pointer[j+1]=*pix_ptr++;
      row_pointer[j+2]=*pix_ptr++;
    }
  }
}

void SimKinect::StartRecord(char* save_name) {
	XnStatus nRetVal;
	nRetVal = recorder.Create(context);
	CHECK_RC(nRetVal, "Create recorder");

	nRetVal = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, save_name);
	CHECK_RC(nRetVal, "Set recorder destination file");

	if ( depth_generator.IsValid() ) {
	// add depth node to recorder
		nRetVal = recorder.AddNodeToRecording(depth_generator);
		CHECK_RC(nRetVal, "Add depth node to recording");
	}
	if ( color_generator.IsValid() ) {
	// add color node to recorder
		nRetVal = recorder.AddNodeToRecording(color_generator);
		CHECK_RC(nRetVal, "Add color node to recording");
	}
	/*
	if ( user_generator.IsValid() ) {
	// add user node to recorder
		nRetVal = recorder.AddNodeToRecording(user_generator);
		CHECK_RC(nRetVal, "Add user node to recording");
	}
	*/
	if ( ir_generator.IsValid() ) {
		// add ir node to recorder
		nRetVal = recorder.AddNodeToRecording(ir_generator);
		CHECK_RC(nRetVal, "Add ir node to recording");
	}
	if ( audio_generator.IsValid() ) {
		// add audio node to recorder
		nRetVal = recorder.AddNodeToRecording(audio_generator);
		CHECK_RC(nRetVal, "Add audio node to recording");
	}
}

void SimKinect::StopRecord() {
	recorder.Release();
}

//get Users
void SimKinect::GetJoint( XnUserID player, XnSkeletonJoint eJoint , SKPoint3D& rjoint, SKPoint2D& pjoint) {
	rjoint.x = rjoint.y = rjoint.z = 0;
	pjoint.x = pjoint.y = 0;
	if (!user_generator.GetSkeletonCap().IsJointActive(eJoint) )	{
		return;
	}

	XnSkeletonJointPosition xnjoint;
	user_generator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, xnjoint);

	if (xnjoint.fConfidence < 0.5)
	{
		return;
	}
	XnPoint3D pt;
	pt = xnjoint.position;
	rjoint.x = pt.X;
	rjoint.y = pt.Y;
	rjoint.z = pt.Z;
	depth_generator.ConvertRealWorldToProjective(1, &pt, &pt);
	pjoint.x = pt.X;
	pjoint.y = pt.Y;
}
void SimKinect::GetUsers(vector<SKUser>& users ) {

	if ( !user_generator.IsValid() ) return ;

	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	user_generator.GetUsers(aUsers, nUsers);
	SKPoint3D rjoint;
	SKPoint2D pjoint;
	XnPoint3D com;
	SKUser user;
	for (int i = 0; i < nUsers; i++) {
		if (!user_generator.GetSkeletonCap().IsTracking(aUsers[i])) continue;
		user_generator.GetCoM(aUsers[i], com);
		rjoint.x = com.X;
		rjoint.y = com.Y;
		rjoint.z = com.Z;
		depth_generator.ConvertRealWorldToProjective(1, &com, &com);
		pjoint.x = com.X;
		pjoint.y = com.Y;
		user.userId=aUsers[i];
		user.real_joints[0] = rjoint;
		user.proj_joints[0] = pjoint;

		GetJoint(aUsers[i],XN_SKEL_HEAD,user.real_joints[1],user.proj_joints[1]);
		GetJoint(aUsers[i],XN_SKEL_NECK,user.real_joints[2],user.proj_joints[2]);
		GetJoint(aUsers[i],XN_SKEL_TORSO,user.real_joints[3],user.proj_joints[3]);
		GetJoint(aUsers[i],XN_SKEL_WAIST,user.real_joints[4],user.proj_joints[4]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_COLLAR,user.real_joints[5],user.proj_joints[5]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_SHOULDER,user.real_joints[6],user.proj_joints[6]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_ELBOW,user.real_joints[7],user.proj_joints[7]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_WRIST,user.real_joints[8],user.proj_joints[8]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_HAND,user.real_joints[9],user.proj_joints[9]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_FINGERTIP,user.real_joints[10],user.proj_joints[10]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_COLLAR,user.real_joints[11],user.proj_joints[11]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_SHOULDER,user.real_joints[12],user.proj_joints[12]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_ELBOW,user.real_joints[13],user.proj_joints[13]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_WRIST,user.real_joints[14],user.proj_joints[14]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_HAND,user.real_joints[15],user.proj_joints[15]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_FINGERTIP,user.real_joints[16],user.proj_joints[16]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_HIP,user.real_joints[17],user.proj_joints[17]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_KNEE,user.real_joints[18],user.proj_joints[18]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_ANKLE,user.real_joints[19],user.proj_joints[19]);
		GetJoint(aUsers[i],XN_SKEL_LEFT_FOOT,user.real_joints[20],user.proj_joints[20]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_HIP,user.real_joints[21],user.proj_joints[21]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_KNEE,user.real_joints[22],user.proj_joints[22]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_ANKLE,user.real_joints[23],user.proj_joints[23]);
		GetJoint(aUsers[i],XN_SKEL_RIGHT_FOOT,user.real_joints[24],user.proj_joints[24]);

		users.push_back(user);
	}
}

void DrawUser ( const SKUser& user , cv::Mat& image) {
		char strLabel[255];
		xnOSMemSet(strLabel, 0, sizeof(strLabel));
		sprintf(strLabel, "User %d", user.userId);
		cv::putText(image,strLabel,cvPoint(user.proj_joints[0].x,user.proj_joints[0].y),CV_FONT_HERSHEY_SIMPLEX,1,CV_RGB(255,255,255),2);
    
		
		for ( int i=1 ; i<SK_JOINT_NUMBER; i++ ){
			cv::circle(image,cvPoint(user.proj_joints[i].x,user.proj_joints[i].y),1,CV_RGB(0,0,255));
		}
}