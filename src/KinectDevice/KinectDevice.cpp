
#include "KinectDevice.h"

using namespace Ogre;
using namespace Kinect;

/*calibrating the depth camera http://openkinect.org/wiki/Imaging_Information
  approximation is given by St¨¦phane Magnenat in this post: distance = 0.1236 * tan(rawDisparity / 2842.5 + 1.1863) in meters. 
  Adding a final offset term of -0.037 centers the original ROS data. The tan approximation has a sum squared difference of .33 cm while the 1/x approximation is about 1.7 cm.
  Once you have the distance using the measurement above, a good approximation for converting (i, j, z) to (x,y,z) is:
		x = (i - w / 2) * (z + minDistance) * scaleFactor * (w/h)
		y = (j - h / 2) * (z + minDistance) * scaleFactor
		z = z
		Where
		minDistance = -10
		scaleFactor = .0021.
		These values were found by hand.
*/
void KinectDevice::RawDepthToMeters1(void)
{
	const float k1 = 1.1863;
    const float k2 = 2842.5;
    const float k3 = 0.1236;
	for (int i=0; i<2048; i++)
	{
        const float depth = k3 * tanf(i/k2 + k1);
		mGammaMap[i]=depth;
	}
}

void KinectDevice::RawDepthToMeters2(void)
{
	for (int i=0; i<2048; i++)
        mGammaMap[i] = float(1.0 / (double(i) * -0.0030711016 + 3.3309495161));
}

void KinectDevice::RawDepthToMeters3(void)
{
	for (int i=0; i<2048; i++)
		mGammaMap[i] = (unsigned short)(float)(powf(i/2048.0f, 3)*6*6*256);
}

KinectDevice::KinectDevice()
{
	mColorTexture.setNull();
	mDepthTexture.setNull();
	mColoredDepthTexture.setNull();
	memset(mDepthBuffer,0,KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT);
	memset(mColorBuffer,0,KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3);
	memset(mUserBuffer,0,KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3);
	memset(mColoredDepthBuffer,0,KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 3);
	memset(PalletIntsR,0,256*sizeof(XnUInt8));
	memset(PalletIntsG,0,256*sizeof(XnUInt8));
	memset(PalletIntsB,0,256*sizeof(XnUInt8));
	mColorTextureAvailable = false;
	mDepthTextureAvailable = false;
	mColoredDepthTextureAvailable = false;

	mColoredDepthPixelBox = Ogre::PixelBox(KINECT_DEPTH_WIDTH, KINECT_DEPTH_HEIGHT, 1, Ogre::PF_R8G8B8, mColoredDepthBuffer);
	mDepthPixelBox = Ogre::PixelBox(KINECT_DEPTH_WIDTH, KINECT_DEPTH_HEIGHT, 1, Ogre::PF_L8, mDepthBuffer);
	mColorPixelBox = Ogre::PixelBox(KINECT_COLOR_WIDTH, KINECT_COLOR_HEIGHT, 1, Ogre::PF_B8G8R8, mColorBuffer);

	m_hUserCallbacks = NULL;
	m_hPoseCallbacks = NULL;
	m_hCalibrationCallbacks = NULL;
	m_pPrimary = NULL;
	mIsWorking=false; 

	RawDepthToMeters1();
	CreateRainbowPallet();

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
		rc = m_Context.InitFromXmlFile("..\\..\\Data\\openni.xml");
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
	if (!mIsWorking)
		return false;
	//get meta data from kinect
	readFrame();
	//parse data to texture
	ParseUserTexture(&sceneMetaData, true);
	ParseColorDepthData(&depthMetaData,&sceneMetaData,&imageMetaData);
	ParseColoredDepthData(&depthMetaData,DepthColoringType::COLOREDDEPTH);
	Parse3DDepthData(&depthMetaData);
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
void KinectDevice::ParseColoredDepthData(xn::DepthMetaData *depthMetaData,DepthColoringType DepthColoring)
{
	unsigned short *tmpGrayPixels = (unsigned short *)depthMetaData->Data();
	int MaxDepth = getDepthGenerator()->GetDeviceMaxDepth();
	int i=0;
	//XnUInt8 nAlpha = m_DrawConfig.Streams.Depth.fTransparency*255;

	XnUInt16 nColIndex;
	for (int y=0; y<480; y++)
	{
		unsigned char* destrow = mColoredDepthBuffer + (y*(640))*3;
		for (int x=0; x<640; x++)
		{
			XnUInt8 nRed = 0;
			XnUInt8 nGreen = 0;
			XnUInt8 nBlue = 0;
			unsigned short Depth = tmpGrayPixels[i];
			switch (DepthColoring)
			{
				case LINEAR_HISTOGRAM:
					nRed = nGreen = depthHist[Depth]*255;
					break;
				case PSYCHEDELIC:
					switch ((Depth/10) % 10)
					{
					case 0:
						nRed = 255;
						break;
					case 1:
						nGreen = 255;
						break;
					case 2:
						nBlue = 255;
						break;
					case 3:
						nRed = 255;
						nGreen = 255;
						break;
					case 4:
						nGreen = 255;
						nBlue = 255;
						break;
					case 5:
						nRed = 255;
						nBlue = 255;
						break;
					case 6:
						nRed = 255;
						nGreen = 255;
						nBlue = 255;
						break;
					case 7:
						nRed = 127;
						nBlue = 255;
						break;
					case 8:
						nRed = 255;
						nBlue = 127;
						break;
					case 9:
						nRed = 127;
						nGreen = 255;
						break;
					}
					break;
				case RAINBOW:
					nColIndex = (XnUInt16)((Depth / (MaxDepth / 256.)));
					nRed = PalletIntsR[nColIndex];
					nGreen = PalletIntsG[nColIndex];
					nBlue = PalletIntsB[nColIndex];
					break;
				case CYCLIC_RAINBOW:
					nColIndex = (Depth % 256);
					nRed = PalletIntsR[nColIndex];
					nGreen = PalletIntsG[nColIndex];
					nBlue = PalletIntsB[nColIndex];
					break;
				case CYCLIC_RAINBOW_HISTOGRAM:{
					float fHist = depthHist[Depth];
					nColIndex = (Depth % 256);
					nRed = PalletIntsR[nColIndex]   * fHist;
					nGreen = PalletIntsG[nColIndex] * fHist;
					nBlue = PalletIntsB[nColIndex]  * fHist;
					break;
					}
				case COLOREDDEPTH:	
					unsigned long pval = mGammaMap[Depth];
					int lb = pval & 0xff;
					switch (pval>>8) 
					{
						case 0:
							nRed = 255;
							nGreen = 255-lb;
							nBlue = 255-lb;
							break;
						case 1:
							nRed = 255;
							nGreen = lb;
							nBlue = 0;
							break;
						case 2:
							nRed = 255-lb;
							nGreen = 255;
							nBlue = 0;
							break;
						case 3:
							nRed = 0;
							nGreen = 255;
							nBlue = lb;
							break;
						case 4:
							nRed = 0;
							nGreen = 255-lb;
							nBlue = 255;
							break;
						case 5:
							nRed = 0;
							nGreen = 0;
							nBlue = 255-lb;
							break;
						default:
							nRed = 0;
							nGreen = 0;
							nBlue = 0;
							break;
					}
			}
			destrow[0] = nBlue;
			destrow[1] = nGreen;
			destrow[2] = nRed;
			destrow += 3;
			i++;
		}
	}
	
	//copy?? the data to pixelbox
	mColoredDepthTextureAvailable = true;	
}

void KinectDevice::CalculateHistogram()
{
	xn::DepthGenerator* pDepthGen = getDepthGenerator();

	if (pDepthGen == NULL)
		return;

	XnUInt32 nZRes = pDepthGen->GetDeviceMaxDepth() + 1;
	xnOSMemSet(depthHist, 0, nZRes*sizeof(float));
	int nNumberOfPoints = 0;

	XnDepthPixel nValue;

	const XnDepthPixel* pDepth = pDepthGen->GetDepthMap();
	const XnDepthPixel* pDepthEnd = pDepth + (pDepthGen->GetDataSize() / sizeof(XnDepthPixel));

	while (pDepth != pDepthEnd)
	{
		nValue = *pDepth;

		XN_ASSERT(nValue <= nZRes);

		if (nValue != 0)
		{
			depthHist[nValue]++;
			nNumberOfPoints++;
		}

		pDepth++;
	}

	XnUInt32 nIndex;
	for (nIndex = 1; nIndex < nZRes; nIndex++)
	{
		depthHist[nIndex] += depthHist[nIndex-1];
	}
	for (nIndex = 1; nIndex < nZRes; nIndex++)
	{
		if (depthHist[nIndex] != 0)
		{
			depthHist[nIndex] = (nNumberOfPoints-depthHist[nIndex]) / nNumberOfPoints;
		}
	}
}
// --------------------------------
// Code
// --------------------------------
void KinectDevice::CreateRainbowPallet()
{
	unsigned char r, g, b;
	for (int i=1; i<255; i++)
	{
		if (i<=29)
		{
			r = (unsigned char)(129.36-i*4.36);
			g = 0;
			b = (unsigned char)255;
		}
		else if (i<=86)
		{
			r = 0;
			g = (unsigned char)(-133.54+i*4.52);
			b = (unsigned char)255;
		}
		else if (i<=141)
		{
			r = 0;
			g = (unsigned char)255;
			b = (unsigned char)(665.83-i*4.72);
		}
		else if (i<=199)
		{
			r = (unsigned char)(-635.26+i*4.47);
			g = (unsigned char)255;
			b = 0;
		}
		else
		{
			r = (unsigned char)255;
			g = (unsigned char)(1166.81-i*4.57);
			b = 0;
		}

		PalletIntsR[i] = r;
		PalletIntsG[i] = g;
		PalletIntsB[i] = b;
	}
}
void KinectDevice::Parse3DDepthData(xn::DepthMetaData * depthMetaData)
{
	unsigned short *tmpGrayPixels = (unsigned short *)depthMetaData->Data();
	int MaxDepth = getDepthGenerator()->GetDeviceMaxDepth();
	// We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
	int skip = 1;
	for (int y=0; y<Kinect::KINECT_DEPTH_HEIGHT; y+=skip)
	{
		unsigned char* destrow = mColoredDepthBuffer + (y*(KINECT_DEPTH_WIDTH))*3;
		for (int x=0; x<Kinect::KINECT_DEPTH_WIDTH; x+=skip)
		{
		    int offset = x+y*KINECT_DEPTH_WIDTH;
			// Convert kinect data to world xyz coordinate
			unsigned short rawDepth = tmpGrayPixels[offset];
			Vector3 v = DepthToWorld(x,y,rawDepth);
			destrow[2] = v[0];
			destrow[1] = v[1];
			destrow[0] = v[2];
			destrow += 3;
		}
	}
}

Vector3 KinectDevice::DepthToWorld(int x, int y, int depthValue)
{
    static const double fx_d = 1.0 / 5.9421434211923247e+02;
    static const double fy_d = 1.0 / 5.9104053696870778e+02;
    static const double cx_d = 3.3930780975300314e+02;
    static const double cy_d = 2.4273913761751615e+02;

    Vector3 result;
    const double depth = mGammaMap[depthValue];;
    result[0] = float((x - cx_d) * depth * fx_d);
    result[1] = float((y - cy_d) * depth * fy_d);
    result[2] = float(depth);
    return result;
}
/*
Point2i KinectDevice::WorldToColor(const Vec3f &pt)
{
    static const Matrix4 rotationMatrix(
                            Vec3f(9.9984628826577793e-01f, 1.2635359098409581e-03f, -1.7487233004436643e-02f),
                            Vec3f(-1.4779096108364480e-03f, 9.9992385683542895e-01f, -1.2251380107679535e-02f),
                            Vec3f(1.7470421412464927e-02f, 1.2275341476520762e-02f, 9.9977202419716948e-01f));
    static const Vec3f translation(1.9985242312092553e-02f, -7.4423738761617583e-04f, -1.0916736334336222e-02f);
    static const Matrix4 finalMatrix = rotationMatrix.Transpose() * Matrix4::Translation(-translation);
    
    static const double fx_rgb = 5.2921508098293293e+02;
    static const double fy_rgb = 5.2556393630057437e+02;
    static const double cx_rgb = 3.2894272028759258e+02;
    static const double cy_rgb = 2.6748068171871557e+02;

    const Vec3f transformedPos = finalMatrix.TransformPoint(pt);
    const float invZ = 1.0f / transformedPos.z;

    Point2i result;
    result.x = Utility::Bound(cv::Math::Round((transformedPos.x * fx_rgb * invZ) + cx_rgb), 0, 639);
    result.y = Utility::Bound(Math::Round((transformedPos.y * fy_rgb * invZ) + cy_rgb), 0, 479);
    return result;
}
*/
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

			mUserBuffer[i * 3 + 0] = 255 * oniColors[nColorID][0];
			mUserBuffer[i * 3 + 1] = 255 * oniColors[nColorID][1];
			mUserBuffer[i * 3 + 2] = 255 * oniColors[nColorID][2];
		}
		else 
		{
			mDepthBuffer[i] = 0;

			mUserBuffer[i * 3 + 0] = 0;
			mUserBuffer[i * 3 + 1] = 0;
			mUserBuffer[i * 3 + 2] = 0;
		}
	}

	const XnRGB24Pixel* pImageRow = imageMetaData->RGB24Data(); // - g_imageMD.YOffset();

	for (XnUInt y = 0; y < Kinect::colorHeight; ++y)
	{
		const XnRGB24Pixel* pImage = pImageRow; // + g_imageMD.XOffset();

		for (XnUInt x = 0; x < Kinect::colorWidth; ++x, ++pImage)
		{
			int index = (y*Kinect::colorWidth + x)*3;
			mColorBuffer[index + 2] = (unsigned char) pImage->nBlue;
			mColorBuffer[index + 1] = (unsigned char) pImage->nGreen;
			mColorBuffer[index + 0] = (unsigned char) pImage->nRed;
		}
		pImageRow += Kinect::colorWidth;
	}

	//copy the data to pixelbox
	mDepthTextureAvailable = true;	
	mColorTextureAvailable = true;
	mUserTextureAvailable = true;
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
			UserTextureName, // name
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
				materialName, // name
				ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(UserTextureName);
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
			depthTextureName, 
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
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->setAlphaRejectSettings(CMPF_GREATER, 127);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(depthTextureName);
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
			colorTextureName, 
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
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(colorTextureName);
		material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureRotate(Ogre::Degree(180)); 
	}
}
//create ColorDepth texture
void KinectDevice::createOgreColoredDepthTexture(const std::string coloredDepthTextureName,const std::string materialName)
{
	if(!coloredDepthTextureName.empty())
	{
		mColoredDepthTexture = TextureManager::getSingleton().createManual(
		coloredDepthTextureName, 
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
		Ogre::MaterialPtr material = MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->createTextureUnitState(coloredDepthTextureName);
		//material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureRotate(Ogre::Degree(180)); 
	}
}
/*
void KinectDevice::drawColorImage()
{
	if (g_DrawConfig.Streams.bBackground)
		TextureMapDraw(&g_texBackground, pLocation);

	const xn::MapMetaData* pImageMD;
	const XnUInt8* pImage = NULL;

	pImageMD = getImageMetaData();
	pImage = getImageMetaData()->Data();

	if (pImageMD->FrameID() == 0)
	{
		return;
	}

	const xn::DepthMetaData* pDepthMetaData = getDepthMetaData();

	for (XnUInt16 nY = pImageMD->YOffset(); nY < pImageMD->YRes() + pImageMD->YOffset(); nY++)
	{
		XnUInt8* pTexture = TextureMapGetLine(&g_texImage, nY) + pImageMD->XOffset()*4;

		if (pImageMD->PixelFormat() == XN_PIXEL_FORMAT_YUV422)
		{
			YUV422ToRGB888(pImage, pTexture, pImageMD->XRes()*2, g_texImage.Size.X*g_texImage.nBytesPerPixel);
			pImage += pImageMD->XRes()*2;
		}
		else
		{
			for (XnUInt16 nX = 0; nX < pImageMD->XRes(); nX++, pTexture+=4)
			{
				XnInt32 nDepthIndex = 0;

				if (pDepthMetaData != NULL)
				{
					XnDouble dRealX = (nX + pImageMD->XOffset()) / (XnDouble)pImageMD->FullXRes();
					XnDouble dRealY = nY / (XnDouble)pImageMD->FullYRes();

					XnUInt32 nDepthX = dRealX * pDepthMetaData->FullXRes() - pDepthMetaData->XOffset();
					XnUInt32 nDepthY = dRealY * pDepthMetaData->FullYRes() - pDepthMetaData->YOffset();

					if (nDepthX >= pDepthMetaData->XRes() || nDepthY >= pDepthMetaData->YRes())
					{
						nDepthIndex = -1;
					}
					else
					{
						nDepthIndex = nDepthY*pDepthMetaData->XRes() + nDepthX;
					}
				}

				switch (pImageMD->PixelFormat())
				{
				case XN_PIXEL_FORMAT_RGB24:
					pTexture[0] = pImage[0];
					pTexture[1] = pImage[1];
					pTexture[2] = pImage[2];
					pImage+=3; 
					break;
				case XN_PIXEL_FORMAT_GRAYSCALE_8_BIT:
					pTexture[0] = pTexture[1] = pTexture[2] = *pImage;
					pImage+=1; 
					break;
				case XN_PIXEL_FORMAT_GRAYSCALE_16_BIT:
					XnUInt16* p16 = (XnUInt16*)pImage;
					pTexture[0] = pTexture[1] = pTexture[2] = (*p16) >> 2;
					pImage+=2; 
					break;
				}

				// decide if pixel should be lit or not
				if (g_DrawConfig.Streams.Image.Coloring == DEPTH_MASKED_IMAGE &&
					(pDepthMetaData == NULL || nDepthIndex == -1 || pDepthMetaData->Data()[nDepthIndex] == 0))
				{
					pTexture[3] = 0;
				}
				else
				{
					pTexture[3] = 255;
				}
			}
		}
	}

	TextureMapUpdate(&g_texImage);
	TextureMapDraw(&g_texImage, pLocation);
}

void KinectDevice::DrawGLUTDepthMapTexture()
{
    XnUInt16 g_nXRes; 
    XnUInt16 g_nYRes; 
    GetImageRes(g_nXRes,g_nYRes);

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
	if (m_UserGenerator.IsValid())
	{
		m_UserGenerator.GetUserPixels(0, sceneMetaData);
	}
}


