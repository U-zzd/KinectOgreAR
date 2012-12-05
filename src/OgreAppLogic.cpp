#include "OgreAppLogic.h"
#include "OgreApp.h"
#include <Ogre.h>
#include "Chrono.h"
#include "StatsFrameListener.h"
#include <OgrePanelOverlayElement.h>

using namespace Ogre;

OgreAppLogic::OgreAppLogic() : mApplication(0)
{
	// ogre
	mSceneMgr		= 0;
	mViewport		= 0;
	mCamera         = 0;
	mCameraNode     = 0;
	mVideoDevice    = 0;
	mWebcamBufferL8 = 0;
	mObjectNode     = 0;
	mTrackingSystem = 0;
	mStatsFrameListener = 0;
	mAnimState = 0;

	mOISListener.mParent = this;
}

OgreAppLogic::~OgreAppLogic()
{}

// preAppInit
bool OgreAppLogic::preInit(const Ogre::StringVector &commandArgs)
{
	return true;
}

// postAppInit
bool OgreAppLogic::init(void)
{
	createSceneManager();
	createViewport();
	createCamera();
	createScene();
	
	//webcam resolution
	int width  = 320;
	int height = 240;

	const std::string colorTextureName        = "KinectColorTexture";
	const std::string depthTextureName        = "KinectDepthTexture";
	const std::string coloredDepthTextureName = "KinectColoredDepthTexture";

	mKinectDevice->createOgreDepthTexture(depthTextureName,"");
	mKinectDevice->createOgreColoredDepthTexture(coloredDepthTextureName,"");
	//mKinectDevice->setMotorPosition(mKinectMotorPosition);
	createKinectOverlay(colorTextureName, depthTextureName, coloredDepthTextureName);

	mTrackingSystem = new TrackingSystem;

	initTracking(width, height);
	createWebcamPlane(width, height, 45000.0f);	


	mStatsFrameListener = new StatsFrameListener(mApplication->getRenderWindow());
	mApplication->getOgreRoot()->addFrameListener(mStatsFrameListener);
	mStatsFrameListener->showDebugOverlay(true);

	mApplication->getKeyboard()->setEventCallback(&mOISListener);
	mApplication->getMouse()->setEventCallback(&mOISListener);

	return true;
}

bool OgreAppLogic::preUpdate(Ogre::Real deltaTime)
{
	return true;
}

bool OgreAppLogic::update(Ogre::Real deltaTime)
{
	//If there is a new frame available on video device
	if (mKinectDevice->Update())
	{
		//RGB -> Gray conversion
		//Ogre::PixelUtil::bulkPixelConversion(mVideoDevice->getBufferData(), Ogre::PF_B8G8R8, mWebcamBufferL8, Ogre::PF_L8, mVideoDevice->getWidth()*mVideoDevice->getHeight());

		//Create Gray level PixelBox
		//Ogre::PixelBox box(mVideoDevice->getWidth(), mVideoDevice->getHeight(), 1, Ogre::PF_L8, (void*) mWebcamBufferL8);
		//Ogre::PixelBox box(mVideoDevice->getWidth(), mVideoDevice->getHeight(), 1, Ogre::PF_B8G8R8, (void*) mVideoDevice->getBufferData());
		Ogre::PixelBox box(mKinectDevice->getWidth(), mKinectDevice->getHeight(), 1, Ogre::PF_B8G8R8, (void*) mKinectDevice->getKinectColorBufferData());

		//Tracking using ArToolKitPlus
		mTrackingSystem->update(box);

		if (mTrackingSystem->isPoseComputed())
		{
			mObjectNode->setVisible(true);
			mCameraNode->setOrientation(mTrackingSystem->getOrientation());
			mCameraNode->setPosition(mTrackingSystem->getTranslation());
		}
		else
		{
			mObjectNode->setVisible(false);
		}
	}
	if (mAnimState)
		mAnimState->addTime(deltaTime);

	bool result = processInputs(deltaTime);
	return result;
}

void OgreAppLogic::shutdown(void)
{
	mKinectDevice->shutdown();
	mKinectDevice = NULL;

	delete[] mWebcamBufferL8;
	mWebcamBufferL8 = NULL;

	delete mTrackingSystem;
	mTrackingSystem = NULL;

	mApplication->getOgreRoot()->removeFrameListener(mStatsFrameListener);
	delete mStatsFrameListener;
	mStatsFrameListener = 0;
	
	if(mSceneMgr)
		mApplication->getOgreRoot()->destroySceneManager(mSceneMgr);
	mSceneMgr = 0;
}

void OgreAppLogic::postShutdown(void)
{

}

//--------------------------------- Init --------------------------------

void OgreAppLogic::createSceneManager(void)
{
	mSceneMgr = mApplication->getOgreRoot()->createSceneManager(ST_GENERIC, "SceneManager");
}

void OgreAppLogic::createViewport(void)
{
	mViewport = mApplication->getRenderWindow()->addViewport(0);
}

void OgreAppLogic::createCamera(void)
{
	mCamera = mSceneMgr->createCamera("camera");
	mCamera->setNearClipDistance(0.5);
	mCamera->setFarClipDistance(50000);
	mCamera->setPosition(0, 0, 0);
	mCamera->lookAt(0, 0, 1);
	mCamera->setFOVy(Degree(40)); //FOVy camera Ogre = 40?
	mCamera->setAspectRatio((float) mViewport->getActualWidth() / (float) mViewport->getActualHeight());	
	mViewport->setCamera(mCamera);

	mCameraNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("cameraNode");
	mCameraNode->setPosition(0, 1700, 0);
	mCameraNode->lookAt(Vector3(0, 1700, -1), Node::TS_WORLD);
	mCameraNode->attachObject(mCamera);
	mCameraNode->setFixedYawAxis(true, Vector3::UNIT_Y);
}

void OgreAppLogic::createScene(void)
{
	mSceneMgr->setSkyBox(true, "Examples/GridSkyBox");

	Ogre::Entity* ent = mSceneMgr->createEntity("Sinbad.mesh");	//1x1_cube.mesh //Sinbad.mesh //axes.mesh

	mObjectNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("cube");	
	mObjectNode->setOrientation(Quaternion(Degree(90.f), Vector3::UNIT_X));
	Ogre::Real scale = 22;
	mObjectNode->setPosition(0, 0, 5*scale);
	mObjectNode->setScale(Ogre::Vector3::UNIT_SCALE*scale);
	mObjectNode->attachObject(ent);

	// create swords and attach them to sinbad
	Ogre::Entity* sword1 = mSceneMgr->createEntity("SinbadSword1", "Sword.mesh");
	Ogre::Entity* sword2 = mSceneMgr->createEntity("SinbadSword2", "Sword.mesh");
	ent->attachObjectToBone("Sheath.L", sword1);
	ent->attachObjectToBone("Sheath.R", sword2);
	mAnimState = ent->getAnimationState("Dance");
	mAnimState->setLoop(true);
	mAnimState->setEnabled(true);

}

void OgreAppLogic::initTracking(int width, int height)
{
	if (mKinectDeviceManager.size() > 0)
	{
		mKinectDevice = mKinectDeviceManager[0];
		mKinectDevice->initPrimeSensor();
		mKinectDevice->createOgreColorTexture("WebcamTexture","");
		mWebcamBufferL8 = new unsigned char[width*height];
		mTrackingSystem->init(width, height);

		//Create Webcam Material
		MaterialPtr material = MaterialManager::getSingleton().create("WebcamMaterial", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		Ogre::Technique *technique = material->createTechnique();
		technique->createPass();
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->createTextureUnitState("WebcamTexture");
	}
	else
	{
		Ogre::Exception(Ogre::Exception::ERR_INVALID_STATE, "No Kinect found", "AppLogic");
	}
}

void OgreAppLogic::createWebcamPlane(int width, int height, Ogre::Real _distanceFromCamera)
{
	// Create a prefab plane dedicated to display video
	float videoAspectRatio = width / (float) height;

	float planeHeight = 2 * _distanceFromCamera * Ogre::Math::Tan(Degree(26)*0.5); //FOVy webcam = 26?(intrinsic param)
	float planeWidth = planeHeight * videoAspectRatio;

	Plane p(Vector3::UNIT_Z, 0.0);
	MeshManager::getSingleton().createPlane("VerticalPlane", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, p , planeWidth, planeHeight, 1, 1, true, 1, 1, 1, Vector3::UNIT_Y);
	Entity* planeEntity = mSceneMgr->createEntity("VideoPlane", "VerticalPlane"); 
	planeEntity->setMaterialName("WebcamMaterial");
	planeEntity->setRenderQueueGroup(RENDER_QUEUE_WORLD_GEOMETRY_1);

	// Create a node for the plane, inserts it in the scene
	Ogre::SceneNode* node = mCameraNode->createChildSceneNode("planeNode");
	node->attachObject(planeEntity);

	// Update position    
	Vector3 planePos = mCamera->getPosition() + mCamera->getDirection() * _distanceFromCamera;
	node->setPosition(planePos);

	// Update orientation
	node->setOrientation(mCamera->getOrientation());
}

void OgreAppLogic::createKinectOverlay(const std::string& colorTextureName, const std::string& depthTextureName, const std::string& coloredDepthTextureName)
{
	//Create Color Overlay
	/*
	{
		//Create Overlay
		Ogre::OverlayManager& overlayManager = Ogre::OverlayManager::getSingleton();
		Ogre::Overlay* overlay = overlayManager.create("KinectColorOverlay");

		//Create Material
		const std::string materialName = "KinectColorMaterial";
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(colorTextureName);

		//Create Panel
		Ogre::PanelOverlayElement* panel = static_cast<Ogre::PanelOverlayElement*>(overlayManager.createOverlayElement("Panel", "KinectColorPanel"));
		panel->setMetricsMode(Ogre::GMM_PIXELS);
		panel->setMaterialName(materialName);
		panel->setDimensions((float)Ogre::Kinect::colorWidth, (float)Ogre::Kinect::colorHeight);
		panel->setPosition(640.0f, 0.0f);
		overlay->add2D(panel);		
		overlay->setZOrder(300);
		overlay->show(); 
	}
	*/
	//Create Depth Overlay
	{
		//Create Overlay
		Ogre::OverlayManager& overlayManager = Ogre::OverlayManager::getSingleton();
		Ogre::Overlay* overlay = overlayManager.create("KinectDepthOverlay");

		//Create Material
		const std::string materialName = "KinectDepthMaterial";
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->setAlphaRejectSettings(CMPF_GREATER, 127);

		material->getTechnique(0)->getPass(0)->createTextureUnitState(depthTextureName);
		material->getTechnique(0)->getPass(0)->setVertexProgram("Ogre/Compositor/StdQuad_vp");
		material->getTechnique(0)->getPass(0)->setFragmentProgram("KinectDepth");

		//Create Panel
		Ogre::PanelOverlayElement* panel = static_cast<Ogre::PanelOverlayElement*>(overlayManager.createOverlayElement("Panel", "KinectDepthPanel"));
		panel->setMetricsMode(Ogre::GMM_PIXELS);
		panel->setMaterialName(materialName);
		panel->setDimensions((float)Kinect::depthWidth, (float)Kinect::depthHeight);
		panel->setPosition((float)640.0f, 0.0f);
		overlay->add2D(panel);		
		overlay->setZOrder(310);
		overlay->show();
	}

	//Create Colored Depth Overlay
	{
		//Create Overlay
		Ogre::OverlayManager& overlayManager = Ogre::OverlayManager::getSingleton();
		Ogre::Overlay* overlay = overlayManager.create("KinectColoredDepthOverlay");

		//Create Material
		const std::string materialName = "KinectColoredDepthMaterial";
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(coloredDepthTextureName);

		//Create Panel
		Ogre::PanelOverlayElement* panel = static_cast<Ogre::PanelOverlayElement*>(overlayManager.createOverlayElement("Panel", "KinectColoredDepthPanel"));
		panel->setMetricsMode(Ogre::GMM_PIXELS);
		panel->setMaterialName(materialName);
		panel->setDimensions((float)Kinect::depthWidth, (float)Kinect::depthHeight);
		panel->setPosition((float)0.0f, 0.0f);
		overlay->add2D(panel);		
		overlay->setZOrder(320);
		overlay->show();
	}
}

//--------------------------------- update --------------------------------

bool OgreAppLogic::processInputs(Ogre::Real deltaTime)
{
	OIS::Keyboard *keyboard = mApplication->getKeyboard();
	if(keyboard->isKeyDown(OIS::KC_ESCAPE))
	{
		return false;
	}

	return true;
}

bool OgreAppLogic::OISListener::mouseMoved( const OIS::MouseEvent &arg )
{
	return true;
}

bool OgreAppLogic::OISListener::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
	return true;
}

bool OgreAppLogic::OISListener::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
	return true;
}

bool OgreAppLogic::OISListener::keyPressed( const OIS::KeyEvent &arg )
{
	return true;
}

bool OgreAppLogic::OISListener::keyReleased( const OIS::KeyEvent &arg )
{
	return true;
}