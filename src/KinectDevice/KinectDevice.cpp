
#include "KinectDevice.h"

using namespace Ogre;
using namespace Kinect;

KinectDevice::KinectDevice()
{
	for (int i=0; i<2048; i++)
		mGammaMap[i] = (unsigned short)(float)(powf(i/2048.0f, 3)*6*6*256);

	mColorTexture.setNull();
	mDepthTexture.setNull();
	mColoredDepthTexture.setNull();
	mColorTextureAvailable = false;
	mDepthTextureAvailable = false;
	mColoredDepthTextureAvailable = false;
	m_hUserCallbacks = NULL;
	m_hPoseCallbacks = NULL;
	m_hCalibrationCallbacks = NULL;
	mIsWorking=false; 
	//init Kinect or Xtion
	//initPrimeSensor();
}

KinectDevice::~KinectDevice()
{
	shutdown();
}

//init Kinect or Xtion
XnStatus KinectDevice::initPrimeSensor()
{
		// Init OpenNI from XML
		XnStatus rc = XN_STATUS_OK;
		rc = m_Context.InitFromXmlFile(".\\Data\\openni.xml");
		CHECK_RC(rc, "InitFromXml");
		// Make sure we have all OpenNI nodes we will be needing for this sample
		xn::NodeInfoList nodes;

#if SHOW_DEPTH
		VALIDATE_GENERATOR(XN_NODE_TYPE_DEPTH, "Depth", m_DepthGenerator);
#endif 
		VALIDATE_GENERATOR(XN_NODE_TYPE_USER, "User", m_UserGenerator);
		VALIDATE_GENERATOR(XN_NODE_TYPE_IMAGE, "Image", m_ImageGenerator);
		VALIDATE_GENERATOR(XN_NODE_TYPE_HANDS, "Gesture", m_GestureGenerator);
		VALIDATE_GENERATOR(XN_NODE_TYPE_HANDS, "Hands", m_HandsGenerator);
		
		// Init NITE Controls (UI stuff)
		m_pSessionManager = new XnVSessionManager;
		rc = m_pSessionManager->Initialize(&m_Context, "Click", "RaiseHand");
		m_pSessionManager->SetQuickRefocusTimeout(0);

		// Init OpenNI nodes and register callback Procedure to OpenNI
		m_HandsGenerator.SetSmoothing(1);
		//m_UserGenerator.RegisterUserCallbacks(SinbadCharacterController::NewUser, SinbadCharacterController::LostUser, this, m_hUserCallbacks);
		//m_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(SinbadCharacterController::PoseDetected, SinbadCharacterController::PoseLost, this, m_hPoseCallbacks);
#if SHOW_DEPTH
		m_DepthGenerator.GetMirrorCap().SetMirror(m_front);
#endif
		// Make sure OpenNI nodes start generating
		rc = m_Context.StartGeneratingAll();
		CHECK_RC(rc, "Kinect StartGenerating Context to Ogre");
		if(rc == XN_STATUS_OK)
			mIsWorking=true; 
		m_candidateID = 0;
		m_pStartPoseDetector = new StartPoseDetector(3.0);
		m_pEndPoseDetector = new EndPoseDetector(m_UserGenerator, 2.0);
		m_pEndPoseDetector->SetUserId(m_candidateID);

		return XN_STATUS_OK;
}

//update the all buffer and texture from kinect
bool KinectDevice::Update()
{
	//get meta data from kinect
#if SHOW_DEPTH
	m_DepthGenerator.GetMetaData(depthMetaData);
#endif
	m_ImageGenerator.GetMetaData(imageMetaData);
	//m_IRGenerator.GetMetaData(irMetaData);
	m_UserGenerator.GetUserPixels(0, sceneMetaData);
	
	//parse data to texture
	ParseUserTexture(&sceneMetaData, true);
	ParseColorDepthData(&depthMetaData,&sceneMetaData,&imageMetaData);
	ParseColoredDepthData(&depthMetaData);
	return UpdateColorDepthTexture();
}

bool KinectDevice::UpdateColorDepthTexture()
{
	bool updated = false;

	if (!mColorTexture.isNull() && mColorTextureAvailable)
	{
		Ogre::HardwarePixelBufferSharedPtr pixelBuffer = mColorTexture->getBuffer();
		pixelBuffer->blitFromMemory(mColorPixelBox);
		mColorTextureAvailable = false;
		updated = true;
	}
	if (!mDepthTexture.isNull() && mDepthTextureAvailable)
	{
		Ogre::HardwarePixelBufferSharedPtr pixelBuffer = mDepthTexture->getBuffer();
		pixelBuffer->blitFromMemory(mDepthPixelBox);
		mDepthTextureAvailable = false;
		updated = true;
	}
	if (!mColoredDepthTexture.isNull()&& mColoredDepthTextureAvailable)
	{
		Ogre::HardwarePixelBufferSharedPtr pixelBuffer = mColoredDepthTexture->getBuffer();
		pixelBuffer->blitFromMemory(mColoredDepthPixelBox);		
		mColoredDepthTextureAvailable = false;
		updated = true;
	}

	return updated;
}
//Parse sceneMetaData into UserTexture
void KinectDevice::ParseUserTexture(xn::SceneMetaData *sceneMetaData, bool m_front)
{
#if SHOW_DEPTH || SHOW_BAR
	//TexturePtr texture = TextureManager::getSingleton().getByName("MyDepthTexture");
	//TexturePtr texture = TextureManager::getSingleton().getByName("MyDepthTexture2");
	if(mUserTexture.isNull())
		return;
	// Get the pixel buffer
	HardwarePixelBufferSharedPtr pixelBuffer = mUserTexture->getBuffer();
	// Lock the pixel buffer and get a pixel box
	pixelBuffer->lock(HardwareBuffer::HBL_DISCARD); 
	const PixelBox& pixelBox = pixelBuffer->getCurrentLock();
	unsigned char* pDest = static_cast<unsigned char*>(pixelBox.data);

	// Get label map 
	const XnLabel* pUsersLBLs = sceneMetaData->Data();
		
	for (size_t j = 0; j < KINECT_DEPTH_HEIGHT; j++)
	{
		pDest = static_cast<unsigned char*>(pixelBox.data) + j*pixelBox.rowPitch*4;
#if SHOW_DEPTH
		for(size_t i = 0; i < KINECT_DEPTH_WIDTH; i++)
#elif SHOW_BAR
		for(size_t i = 0; i < 50; i++)
#endif
		{
			// fix i if we are mirrored
			uint fixed_i = i;
			if(!m_front)
			{
				fixed_i = KINECT_DEPTH_WIDTH - i;
			}

			// determine color
#if SHOW_DEPTH
			unsigned int color = GetColorForUser(pUsersLBLs[j*KINECT_DEPTH_WIDTH + fixed_i]);

			// if we have a candidate, filter out the rest
			if (m_candidateID != 0)
			{
				if  (m_candidateID == pUsersLBLs[j*KINECT_DEPTH_WIDTH + fixed_i])
				{
					color = GetColorForUser(1);
					if( j > KINECT_DEPTH_HEIGHT*(1 - m_pStartPoseDetector->GetDetectionPercent()))
					{
						//highlight user
						color |= 0xFF070707;
					}
					if( j < KINECT_DEPTH_HEIGHT*(m_pEndPoseDetector->GetDetectionPercent()))
					{	
						//hide user
						color &= 0x20F0F0F0;
					}
				}
				else
				{
					color = 0;
				}
			}
#elif SHOW_BAR
			// RED. kinda.
			unsigned int color = 0x80FF0000;
			if( j > KINECT_DEPTH_HEIGHT*(1 - m_pStartPoseDetector->GetDetectionPercent()))
			{
				//highlight user
				color |= 0xFF070707;
			}
			if( j < KINECT_DEPTH_HEIGHT*(m_pEndPoseDetector->GetDetectionPercent()))
			{	
				//hide user
				color &= 0x20F0F0F0;
			}

			if ((m_pStartPoseDetector->GetDetectionPercent() == 1) ||
				(m_pEndPoseDetector->GetDetectionPercent() == 1))
			{
				color = 0;
			}
#endif
				
			// write to output buffer
			*((unsigned int*)pDest) = color;
			pDest+=4;
		}
	}
	// Unlock the pixel buffer
	pixelBuffer->unlock();
#endif // SHOW_DEPTH
}

//convertDepthToRGB function
void KinectDevice::ParseColoredDepthData(xn::DepthMetaData *depthMetaData)
{
	unsigned short *tmpGrayPixels = (unsigned short *)depthMetaData->Data();
	int i=0;
	for (int y=0; y<480; y++)
	{
		unsigned char* destrow = mColoredDepthBuffer + (y*(640))*3;
		for (int x=0; x<640; x++)
		{
			unsigned short Depth = tmpGrayPixels[i];
			int pval = mGammaMap[Depth];
			int lb = pval & 0xff;
			switch (pval>>8) 
			{
				case 0:
					destrow[2] = 255;
					destrow[1] = 255-lb;
					destrow[0] = 255-lb;
					break;
				case 1:
					destrow[2] = 255;
					destrow[1] = lb;
					destrow[0] = 0;
					break;
				case 2:
					destrow[2] = 255-lb;
					destrow[1] = 255;
					destrow[0] = 0;
					break;
				case 3:
					destrow[2] = 0;
					destrow[1] = 255;
					destrow[0] = lb;
					break;
				case 4:
					destrow[2] = 0;
					destrow[1] = 255-lb;
					destrow[0] = 255;
					break;
				case 5:
					destrow[2] = 0;
					destrow[1] = 0;
					destrow[0] = 255-lb;
					break;
				default:
					destrow[2] = 0;
					destrow[1] = 0;
					destrow[0] = 0;
					break;
			}
			destrow += 3;
			i++;
		}
	}
	
	//copy?? the data to pixelbox
	mColoredDepthPixelBox = Ogre::PixelBox(KINECT_DEPTH_WIDTH, KINECT_DEPTH_HEIGHT, 1, Ogre::PF_R8G8B8, mColoredDepthBuffer);
	mColoredDepthTextureAvailable = true;	
}
	
//Parse color&depth Texture
void KinectDevice::ParseColorDepthData(xn::DepthMetaData *depthMetaData,
							xn::SceneMetaData *sceneMetaData,
							xn::ImageMetaData *imageMetaData)
{
	// Calculate the accumulative histogram
	unsigned int nValue = 0;
	unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
		
	//get the depth data and scene data from kinect
	memset(depthHist, 0, KINECT_MAX_DEPTH*sizeof(float));
	const XnDepthPixel* pDepth = depthMetaData->Data();	

	int n = 0;
	for (nY=0; nY < KINECT_DEPTH_HEIGHT; nY++)
	{
		for (nX=0; nX < KINECT_DEPTH_WIDTH; nX++, nIndex++)
		{
			nValue = pDepth[nIndex];

			if (nValue != 0)
			{
				depthHist[nValue]++;
				nNumberOfPoints++;
			}						
		}
	}
	
	for (nIndex=1; nIndex < KINECT_MAX_DEPTH; nIndex++)
	{
		depthHist[nIndex] += depthHist[nIndex-1];
	}

	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex < KINECT_MAX_DEPTH; nIndex++)
		{
			depthHist[nIndex] = (unsigned int)(256 * (1.0f - (depthHist[nIndex] / nNumberOfPoints)));
		}
	}
	
	const XnLabel* pLabels = sceneMetaData->Data();
	XnLabel label;

	for (int i = 0; i < KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; i++)
	{			
		nValue = pDepth[i];
		label = pLabels[i];
		XnUInt32 nColorID = label % nColors;
		if (label == 0)
		{
			nColorID = nColors;
		}

		if (nValue != 0) 
		{
			nHistValue = depthHist[nValue];
			mDepthBuffer[i] = nHistValue;

			mColorBuffer[i * 3 + 0] = 255 * oniColors[nColorID][0];
			mColorBuffer[i * 3 + 1] = 255 * oniColors[nColorID][1];
			mColorBuffer[i * 3 + 2] = 255 * oniColors[nColorID][2];
		}
		else 
		{
			mDepthBuffer[i] = 0;

			mColorBuffer[i * 3 + 0] = 0;
			mColorBuffer[i * 3 + 1] = 0;
			mColorBuffer[i * 3 + 2] = 0;
		}
	}

	//copy the data to pixelbox
	mDepthPixelBox = Ogre::PixelBox(KINECT_DEPTH_WIDTH, KINECT_DEPTH_HEIGHT, 1, Ogre::PF_L8, mDepthBuffer);
	mDepthTextureAvailable = true;	
	mColorPixelBox = Ogre::PixelBox(KINECT_COLOR_WIDTH, KINECT_COLOR_HEIGHT, 1, Ogre::PF_B8G8R8, mColorBuffer);
	mColorTextureAvailable = true;
}

//create multi screen with dynamic texture
void KinectDevice::createMutliDynamicTexture()
{
	// create the kinect depth image material
	const std::string userTextureName        = "KinectUserTexture";
	const std::string colorTextureName        = "KinectColorTexture";
	const std::string depthTextureName        = "KinectDepthTexture";
	const std::string coloredDepthTextureName = "KinectColoredDepthTexture";
	
	createOgreUserTexture(userTextureName, "KinectUserMatrial");
	createOgreColorTexture(colorTextureName,"KinectColorMaterial");
	createOgreDepthTexture(depthTextureName, "KinectDepthMaterial");
	createOgreColoredDepthTexture(coloredDepthTextureName, "KinectColoredDepthMaterial");
}

//create User texture
void KinectDevice::createOgreUserTexture(const std::string UserTextureName, const std::string materialName)
{
	if(!UserTextureName.empty())
	{		
		mUserTexture  = TextureManager::getSingleton().createManual(
			&UserTextureName, // name
			ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			TEX_TYPE_2D,      // type
			KINECT_DEPTH_WIDTH, KINECT_DEPTH_HEIGHT,// width & height
			0,                // number of mipmaps
			PF_BYTE_BGRA,     // pixel format
			TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
	}
	if(!materialName.empty())
	{
		// Create a material using the texture
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(
				&materialName, // name
				ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(&UserTextureName);
		material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureRotate(Ogre::Degree(180)); 
		//material->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);
	}
}
//create Depth texture
void KinectDevice::createOgreDepthTexture(const std::string depthTextureName,const std::string materialName)
{
	if(!depthTextureName.empty())
	{
		mDepthTexture = TextureManager::getSingleton().createManual(
			&depthTextureName, 
			ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
			TEX_TYPE_2D, 
			KINECT_DEPTH_WIDTH, 
			KINECT_DEPTH_HEIGHT, 
			0,
			PF_L8, 
			TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
	}
	if(!materialName.empty())
	{
		//Create Material
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(&materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->setAlphaRejectSettings(CMPF_GREATER, 127);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(&depthTextureName);
		//material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureRotate(Ogre::Degree(180)); 
		//material->getTechnique(0)->getPass(0)->setVertexProgram("Ogre/Compositor/StdQuad_vp");
		//material->getTechnique(0)->getPass(0)->setFragmentProgram("KinectDepth");
	}
}
//create Color texture
void KinectDevice::createOgreColorTexture(const std::string colorTextureName, const std::string materialName)
{
	if(!colorTextureName.empty())
	{
		mColorTexture = TextureManager::getSingleton().createManual(
			&colorTextureName, 
			ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
			TEX_TYPE_2D, 
			KINECT_DEPTH_WIDTH, 
			KINECT_DEPTH_HEIGHT, 
			0,
			PF_R8G8B8, 
			TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
	}
	if(!materialName.empty())
	{
		//Create Material
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(&materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(&colorTextureName);
		material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureRotate(Ogre::Degree(180)); 
	}
}
//create ColorDepth texture
void KinectDevice::createOgreColoredDepthTexture(const std::string coloredDepthTextureName,const std::string materialName)
{
	if(!coloredDepthTextureName.empty())
	{
		mColoredDepthTexture = TextureManager::getSingleton().createManual(
		&coloredDepthTextureName, 
		ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
		TEX_TYPE_2D, 
		KINECT_DEPTH_WIDTH, 
		KINECT_DEPTH_HEIGHT, 
		0,
		PF_R8G8B8, 
		TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
		//mColoredDepthBuffer   = new unsigned char[Ogre::Kinect::depthWidth * Ogre::Kinect::depthHeight * 3];
		//mColoredDepthPixelBox = Ogre::PixelBox(Ogre::Kinect::depthWidth, Ogre::Kinect::depthHeight, 1, Ogre::PF_R8G8B8, mColoredDepthBuffer);
	}
	if(!materialName.empty())
	{
		//Create Material
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(&materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(&coloredDepthTextureName);
		//material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureRotate(Ogre::Degree(180)); 
	}
}

/*
void KinectDevice::DrawGLUTDepthMapTexture()
{
    XnUInt16 g_nXRes; 
    XnUInt16 g_nYRes; 
    m_pUserTrackerObj->GetImageRes(g_nXRes,g_nYRes);

    if (g_bDrawPixels)
    {
        m_pUserTrackerObj->FillTexture(pDepthTexBuf,texWidth,texHeight,g_bDrawBackground);
    }
    else
    {
        xnOSMemSet(pDepthTexBuf, 0, 3*2*g_nXRes*g_nYRes); // makes the texture empty.
    }

    // makes sure we draw the relevant texture
    glBindTexture(GL_TEXTURE_2D, depthTexID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

    // Display the OpenGL texture map
    glColor4f(0.75,0.75,0.75,1);

    glEnable(GL_TEXTURE_2D);

    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

    GLfloat verts[8] = { g_nXRes, g_nYRes, g_nXRes, 0, 0, 0, 0, g_nYRes };
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    //TODO: Maybe glFinish needed here instead - if there's some bad graphics crap
    glFlush();
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glDisable(GL_TEXTURE_2D);
}
*/

void* KinectDevice::getKinectColorBufferData() const
{
		return (unsigned char *)mColorBuffer;
}

void* KinectDevice::getKinectDepthBufferData() const
{
		return (unsigned char *)mDepthBuffer;
}

void* KinectDevice::getKinectColoredDepthBufferData() const
{
		return (unsigned char *)mColoredDepthBuffer;
}

void KinectDevice::GetImageRes(XnUInt16 &xRes, XnUInt16 &yRes)
{
    xn::DepthMetaData depthMD;
    m_DepthGenerator.GetMetaData(depthMD);
    xRes = (XnUInt16)depthMD.XRes();
    yRes = (XnUInt16)depthMD.YRes();
}

void KinectDevice::closeDevice()
{
	m_Player.Release();
	m_Device.Release();
	m_DepthGenerator.Release();
	m_ImageGenerator.Release();
	m_IRGenerator.Release();
	m_AudioGenerator.Release();
	m_scriptNode.Release();
	m_Context.Release();
}

void KinectDevice::shutdown()
{
	if (mIsWorking)
		closeDevice();
	mIsWorking = false;
	mColorTexture.setNull();
	mDepthTexture.setNull();
	mColoredDepthTexture.setNull();
}
void KinectDevice::readFrame()
{
	XnStatus rc = XN_STATUS_OK;

	if (m_pPrimary != NULL)
	{
		rc = m_Context.WaitOneUpdateAll(*m_pPrimary);
	}
	else
	{
		rc = m_Context.WaitAnyUpdateAll();
	}

	if (rc != XN_STATUS_OK)
	{
		printf("Error: %s\n", xnGetStatusString(rc));
	}

	if (m_DepthGenerator.IsValid())
	{
		m_DepthGenerator.GetMetaData(depthMetaData);
	}

	if (m_ImageGenerator.IsValid())
	{
		m_ImageGenerator.GetMetaData(imageMetaData);
	}

	if (m_IRGenerator.IsValid())
	{
		m_IRGenerator.GetMetaData(irMetaData);
	}

	if (m_AudioGenerator.IsValid())
	{
		m_AudioGenerator.GetMetaData(audioMetaData);
	}
}


