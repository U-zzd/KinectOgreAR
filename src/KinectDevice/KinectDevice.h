#ifndef _KinectDevice
#define _KinectDevice

#include <XnVDeviceGenerator.h>
#include <XnVNite.h>
#include <XnTypes.h>
#include <XnV3DVector.h>
#include "UserSelector.h"
#include "SkeletonPoseDetector.h"
#include "Ogre.h"

namespace Kinect
{
#define SHOW_DEPTH 1
#define SHOW_BAR 0

#define VALIDATE_GENERATOR(type, desc, generator)				\
{																\
	rc = m_Context.EnumerateExistingNodes(nodes, type);			\
	if (nodes.IsEmpty())										\
{															\
	printf("No %s generator!\n", desc);						\
	return 1;												\
}															\
	(*(nodes.Begin())).GetInstance(generator);					\
}
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
{																\
	printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	return rc;													\
}

// Note: wont work as expected for > 5 users in scene
static unsigned int g_UsersColors[] = {/*0x70707080*/0 ,0x80FF0000,0x80FF4500,0x80FF1493,0x8000ff00, 0x8000ced1,0x80ffd700};
#define GetColorForUser(i) g_UsersColors[(i)%(sizeof(g_UsersColors)/sizeof(unsigned int))]

enum
{
	KINECT_DEPTH_WIDTH = 640,
	KINECT_DEPTH_HEIGHT = 480,
	KINECT_COLOR_WIDTH = 640,
	KINECT_COLOR_HEIGHT = 480,
	KINECT_MICROPHONE_COUNT = 4,
	KINECT_AUDIO_BUFFER_LENGTH = 1024,
	KINECT_MAX_DEPTH =10000,
};

static XnFloat oniColors[][3] =
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
	{0,0,0}
};
static XnUInt32 nColors = 10;

static const unsigned int colorWidth        = KINECT_COLOR_WIDTH;
static const unsigned int colorHeight       = KINECT_COLOR_HEIGHT;
static const unsigned int depthWidth        = KINECT_DEPTH_WIDTH;
static const unsigned int depthHeight       = KINECT_DEPTH_HEIGHT;
static const unsigned int nbMicrophone      = KINECT_MICROPHONE_COUNT;
static const unsigned int audioBufferlength = KINECT_AUDIO_BUFFER_LENGTH;


class KinectDevice{
public:
	XnStatus initPrimeSensor(); 
	KinectDevice();	//kinect init for openni
	KinectDevice(void *internalhandle, void *internalmotorhandle);  // takes usb handle.. never explicitly construct! use kinectfinder!
	virtual ~KinectDevice();
	
	//unclasified functions
	bool Opened();
	void SetMotorPosition(double pos);
	void SetLedMode(int NewMode);
	bool GetAcceleroData(float *x, float *y, float *z);
		

	//void AddListener(KinectListener *K);
	//void RemoveListener(KinectListener *K);
	//std::vector<KinectListener *> mListeners;
		
	CRITICAL_SECTION mListenersLock;
		
	void *mInternalData;

	void KinectDisconnected();
	void DepthReceived();
	void ColorReceived();
	void AudioReceived();

	void ParseColorBuffer();
	void ParseDepthBuffer();

	//functions about handle the texture and buffer, not classify yet
	void ParseUserTexture(xn::SceneMetaData *sceneMetaData,	bool m_front);
	void ParseColorDepthData(xn::DepthMetaData *depthMetaData,
							xn::SceneMetaData *sceneMetaData,
							xn::ImageMetaData *imageMetaData);
	void ParseColoredDepthData(xn::DepthMetaData *);
	//create Ogre Texture
	void createMutliDynamicTexture();
	void createOgreUserTexture(const std::string,const std::string);
	void createOgreDepthTexture(const std::string,const std::string);
	void createOgreColorTexture(const std::string,const std::string);
	void createOgreColoredDepthTexture(const std::string,const std::string);
	
	//update the Ogre Texture
	bool Update();  //all buffer and texture is updated
	bool UpdateColorDepthTexture();

	//get depth image Res
	void GetImageRes(XnUInt16 &xRes, XnUInt16 &yRes);

	//read data frame by frame
	void readFrame();
	void closeDevice();
	void shutdown();

	//return the bufferdata
	void* getKinectColorBufferData() const;
	void* getKinectDepthBufferData() const;
	void* getKinectColoredDepthBufferData() const;

	//simple inline functionalities
	int getWidth() const
	{
		return KINECT_DEPTH_WIDTH;
	}

	int getHeight() const
	{
		return KINECT_DEPTH_HEIGHT;
	}

	size_t getBufferSize() const
	{
		return KINECT_DEPTH_WIDTH*KINECT_DEPTH_HEIGHT;
	}

	//normal inline device functions
	xn::Device* getDevice()
	{
		return m_Device.IsValid() ? &m_Device : NULL;
	}
	xn::DepthGenerator* getDepthGenerator()
	{
		return m_DepthGenerator.IsValid() ? &m_DepthGenerator : NULL;
	}
	xn::ImageGenerator* getImageGenerator()
	{
		return m_ImageGenerator.IsValid() ? &m_ImageGenerator : NULL;
	}
	xn::IRGenerator* getIRGenerator()
	{
		return m_IRGenerator.IsValid() ? &m_IRGenerator : NULL;
	}
	xn::AudioGenerator* getAudioGenerator()
	{
		return m_AudioGenerator.IsValid() ? &m_AudioGenerator : NULL;
	}

	const xn::DepthMetaData* getDepthMetaData()
	{
		return m_DepthGenerator.IsValid() ? &depthMetaData : NULL;
	}
	const xn::ImageMetaData* getImageMetaData()
	{
		return m_ImageGenerator.IsValid() ? &imageMetaData : NULL;
	}
	const xn::IRMetaData* getIRMetaData()
	{
		return m_IRGenerator.IsValid() ? &irMetaData : NULL;
	}
	const xn::AudioMetaData* getAudioMetaData()
	{
		return m_AudioGenerator.IsValid() ? &audioMetaData : NULL;
	}

	//normal inline texture functions
	Ogre::TexturePtr getColorTexture()
	{
		return mColorTexture; 
	}
	
	Ogre::TexturePtr getDepthTexture()
	{
		return mDepthTexture; 
	}

	Ogre::TexturePtr getColoredDepthTexture()
	{
		return mColoredDepthTexture; 
	}

	 unsigned char* getColorBuffer()
	{
		return mColorBuffer; 
	}
	
	unsigned char* getDepthBuffer()
	{
		return mDepthBuffer; 
	}

	unsigned char* getColoredDepthBuffer()
	{
		return mColoredDepthBuffer; 
	}
private:

	xn::Device m_Device;
	xn::Player m_Player;
	xn::Context m_Context;
	xn::ScriptNode m_scriptNode;
	bool mIsWorking;

	//xnCallback hands
	XnCallbackHandle m_hPoseCallbacks;
	XnCallbackHandle m_hUserCallbacks;
	XnCallbackHandle m_hCalibrationCallbacks;
	//xnGenerateors
	xn::ProductionNode* m_pPrimary;
#if SHOW_DEPTH
	xn::DepthGenerator m_DepthGenerator;
#endif
	xn::UserGenerator m_UserGenerator;
	xn::ImageGenerator m_ImageGenerator;
	xn::IRGenerator m_IRGenerator;
	xn::HandsGenerator m_HandsGenerator;
	xn::GestureGenerator m_GestureGenerator;
	xn::AudioGenerator m_AudioGenerator;
	xn::SceneAnalyzer m_SceneAnalyzer;

	XnVSessionManager* m_pSessionManager;
	XnUserID m_candidateID;

	//something about the quit slider, not mandatory thing
	XnVFlowRouter* m_pQuitFlow;
	XnVSelectableSlider1D* m_pQuitSSlider;
	
	//skeleton
	double m_SmoothingFactor;
	int m_SmoothingDelta;
	StartPoseDetector * m_pStartPoseDetector;
	EndPoseDetector * m_pEndPoseDetector;
	
	//Kinect MetaData
	xn::SceneMetaData sceneMetaData;
	xn::DepthMetaData depthMetaData;
	xn::ImageMetaData imageMetaData;
	xn::IRMetaData irMetaData;
	xn::AudioMetaData audioMetaData;
	
	//User buffer&texture for Ogre
	Ogre::TexturePtr mUserTexture;
	Ogre::MaterialPtr mUserMaterial;
	Ogre::PixelBox   mUserPixelBox;
	bool             mUserTextureAvailable;
	bool			 m_front;

	//Color RGB buffer&texture for Ogre
	Ogre::TexturePtr mColorTexture;
	Ogre::MaterialPtr mColorMaterial;
	Ogre::PixelBox   mColorPixelBox;
	bool             mColorTextureAvailable;

	//Depth buffer&texture for Ogre
	Ogre::TexturePtr mDepthTexture;
	Ogre::MaterialPtr mDepthMaterial;
	Ogre::PixelBox   mDepthPixelBox;
	bool             mDepthTextureAvailable;

	//Color&depth RGB buffer&texture for Ogre
	Ogre::TexturePtr mColoredDepthTexture;
	Ogre::MaterialPtr mColoredDepthMaterial;	
	Ogre::PixelBox   mColoredDepthPixelBox;
	bool             mColoredDepthTextureAvailable;

	//Kinect Data Buffer
	unsigned short  mGammaMap[2048];
	unsigned char	mDepthBuffer[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT];  //also tempary depth Pixel for Ogre
	unsigned char	mColorBuffer[KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3]; // also tmpeary colore pixel for Ogre
	unsigned char   mColoredDepthBuffer[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT * 3]; //also tempeary colored depth pixel for Ogre
	float mAudioBuffer[KINECT_MICROPHONE_COUNT][KINECT_AUDIO_BUFFER_LENGTH];
	float depthHist[KINECT_MAX_DEPTH];

};

}
#endif