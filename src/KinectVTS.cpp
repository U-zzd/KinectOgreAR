#include "KinectVTS.h"
/*
void createBoxBullet(SceneManager* mSceneMgr,std::string boxName, Vector3 &scaleSize,Vector3 &position,Quaternion& quaternion, float bodyMass,std::string materialName)
	{
		Vector3 size = Vector3::ZERO;	// size of the box
 
 		// create an ordinary, Ogre mesh with texture
 		Entity *entity = mSceneMgr->createEntity(
 				boxName,
 				"cube.mesh");			    
 		entity->setCastShadows(true);
 		// we need the bounding box of the box to be able to set the size of the Bullet-box
 		AxisAlignedBox boundingB = entity->getBoundingBox();
 		size = boundingB.getSize(); size /= 2.0f; // only the half needed
 		size *= 0.95f;	// Bullet margin is a bit bigger so we need a smaller size
 								// (Bullet 2.76 Physics SDK Manual page 18)
 		//entity->setMaterialName("Examples/CloudySky");
		entity->setMaterialName(materialName);
 		SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
 		node->attachObject(entity);
		node->setPosition(position);
		node->setOrientation(quaternion);
 		node->scale(scaleSize);	// the cube is too big for us
 		size *= scaleSize;		// don't forget to scale down the Bullet-box too
		
		
 		// after that create the Bullet shape with the calculated size
 		OgreBulletCollisions::BoxCollisionShape *sceneBoxShape = new OgreBulletCollisions::BoxCollisionShape(size);
 		// and the Bullet rigid body
 		OgreBulletDynamics::RigidBody *defaultBody = new OgreBulletDynamics::RigidBody(
 				boxName+"Rigid", 
 				mWorld);
 		defaultBody->setShape(	node,
 					sceneBoxShape,
 					0.6f,			// dynamic body restitution
 					0.6f,			// dynamic body friction
 					bodyMass, 		// dynamic bodymass
 					position,		// starting position of the box
 					quaternion);// orientation of the box				
 
 		// push the created objects to the dequese
 		mShapes.push_back(sceneBoxShape);
 		mBodies.push_back(defaultBody);				
	}

	*/