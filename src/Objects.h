#pragma once
#include "btBulletDynamicsCommon.h"
#include <vector>
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;


class Objects
{
public:
	
	Objects::Objects();
	Objects::Objects(btRigidBody* objectRigidBody);
	Objects::~Objects();
	btRigidBody* object;
	btCylinderShape* objectShape;
	//btConvexHullShape* oShape;
	btTransform bShapeTrans;
	void Objects::SpawnObjectGroup(std::vector<Objects*> b);
	void Objects::SetPosition(const btVector3 &position);
	void Objects::SetActive();
	btCylinderShape* GetShape() { return objectShape; };
	btTransform GetTransform() { return bShapeTrans; };
};

