#include <XnVDeviceGenerator.h>
#include <XnVNite.h>
#include <XnTypes.h>
#include <XnV3DVector.h>
#include "UserSelector.h"
#include "SkeletonPoseDetector.h"
#include "Ogre.h"
#include "UserSelector.h"
#include "UserTracker.h"
#include "OgreBulletDynamicsRigidBody.h"				 // for OgreBullet
#include "Shapes/OgreBulletCollisionsBoxShape.h"		 // for Boxes
#include "Shapes/OgreBulletCollisionsSphereShape.h"		 // for Boxes

class SinbadCharacterController: UserSelector, UserTracker
{
	// all the animations our character has, and a null ID
	// some of these affect separate body parts and will be blended together
	enum AnimID
	{
		ANIM_IDLE_BASE,
		ANIM_IDLE_TOP,
		ANIM_RUN_BASE,
		ANIM_RUN_TOP,
		ANIM_HANDS_CLOSED,
		ANIM_HANDS_RELAXED,
		ANIM_DRAW_SWORDS,
		ANIM_SLICE_VERTICAL,
		ANIM_SLICE_HORIZONTAL,
		ANIM_DANCE,
		ANIM_JUMP_START,
		ANIM_JUMP_LOOP,
		ANIM_JUMP_END,
		ANIM_NONE
	};

	//float depthHist[MAX_DEPTH];
	xn::SceneMetaData sceneMetaData;
	xn::DepthMetaData depthMetaData;
	xn::ImageMetaData imageMetaData;

	unsigned char * tmpGrayPixels;
	unsigned char * tmpColorPixels;

	//variables from ogrekinect
	unsigned short    mGammaMap[2048];

	Ogre::TexturePtr mColorTexture;
	Ogre::PixelBox   mColorPixelBox;
	bool             mColorTextureAvailable;

	Ogre::TexturePtr mDepthTexture;
	Ogre::PixelBox   mDepthPixelBox;
	bool             mDepthTextureAvailable;

	Ogre::TexturePtr mColoredDepthTexture;
	Ogre::PixelBox   mColoredDepthPixelBox;
	unsigned char*   mColoredDepthBuffer;

public:
	xn::Context m_Context;
#if SHOW_DEPTH
	xn::DepthGenerator m_DepthGenerator;
#endif
	xn::UserGenerator m_UserGenerator;
	xn::ImageGenerator m_ImageGenerator;
	xn::HandsGenerator m_HandsGenerator;
	xn::GestureGenerator m_GestureGenerator;
	xn::SceneAnalyzer m_SceneAnalyzer;

	XnVSessionManager* m_pSessionManager;
	XnVFlowRouter* m_pQuitFlow;
	XnVSelectableSlider1D* m_pQuitSSlider;
	
	double m_SmoothingFactor;
	int m_SmoothingDelta;

	bool m_front;	
	bool suppress;

	OgreBites::ParamsPanel* m_help;
	OgreBites::YesNoSlider* m_quitSlider;
	OgreBites::SdkTrayManager *m_pTrayMgr;

	Vector3 m_origTorsoPos;
	XnUserID m_candidateID;

	StartPoseDetector * m_pStartPoseDetector;
	EndPoseDetector * m_pEndPoseDetector;

	XnCallbackHandle m_hPoseCallbacks;
	XnCallbackHandle m_hUserCallbacks;
	XnCallbackHandle m_hCalibrationCallbacks;
}

static void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie)
{
	// start looking for calibration pose for new users
	generator.GetPoseDetectionCap().StartPoseDetection("Psi", nUserId);
}

static  void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, const XnUserID nUserId, void* pCookie)
{
	SinbadCharacterController* This = (SinbadCharacterController*)pCookie;
	if(This->m_candidateID == nUserId )
	{
		This->m_candidateID = 0;
		This->resetBonesToInitialState();
		This->m_pEndPoseDetector->SetUserId(0);
		This->m_pStartPoseDetector->Reset();
	}
}

static  void XN_CALLBACK_TYPE CalibrationStart(xn::SkeletonCapability& skeleton, const XnUserID nUserId, void* pCookie)
{
}

static  void XN_CALLBACK_TYPE CalibrationEnd(xn::SkeletonCapability& skeleton, const XnUserID nUserId, XnBool bSuccess, void* pCookie)
{
	SinbadCharacterController* This = (SinbadCharacterController*)pCookie;

	if (bSuccess)
	{
		// start tracking
		skeleton.StartTracking(nUserId);
			
		This->m_pStartPoseDetector->SetStartPoseState(true);
		This->m_pEndPoseDetector->SetUserId(nUserId);

		// save torso position
		XnSkeletonJointPosition torsoPos;
		skeleton.GetSkeletonJointPosition(nUserId, XN_SKEL_TORSO, torsoPos);
		This->m_origTorsoPos.x = -torsoPos.position.X;
		This->m_origTorsoPos.y = torsoPos.position.Y;
		This->m_origTorsoPos.z = -torsoPos.position.Z;

		//This->m_pQuitFlow->SetActive(NULL);

		This->suppress = true;
	}
	else
	{
		This->m_candidateID = 0;
	}
}

static void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& poseDetection, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	SinbadCharacterController* This = (SinbadCharacterController*)pCookie;

	// If we dont have an active candidate
	if(This->m_candidateID == 0)
	{
		This->m_candidateID = nId;
		This->m_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		This->m_pStartPoseDetector->SetStartPoseState(true);
	}
}

static void XN_CALLBACK_TYPE PoseLost(xn::PoseDetectionCapability& poseDetection, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	SinbadCharacterController* This = (SinbadCharacterController*)pCookie;
	This->m_pStartPoseDetector->Reset();
}
