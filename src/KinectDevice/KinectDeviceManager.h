#include <string>
#include <vector>
#include <Ogre.h>
#include "KinectDevice.h"

class KinectDeviceManager
{
	public:
		KinectDeviceManager();
		~KinectDeviceManager();
		
		unsigned int size() const;
		KinectDevice* operator[](int index);

	protected:
		KinectDevice* mKinect;
		std::vector<KinectDevice*> mDevices;

	friend class KinectDevice;
};