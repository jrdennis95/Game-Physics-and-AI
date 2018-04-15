#include "Objects.h"

std::vector<Objects*> objectArray;

Objects::Objects()
{
	object = nullptr;
	objectShape = nullptr;
}


Objects::Objects(btRigidBody* boidRigidBody)
{
	object = boidRigidBody;
	objectShape = nullptr;
}
Objects::~Objects() 
{
	for (unsigned int i = 0; i < objectArray.size(); ++i) {
		delete objectArray[i];
	}
	objectArray.clear();
	delete object;
	delete objectShape;
}
void Objects::SetPosition(const btVector3 &position) {
	bShapeTrans = btTransform();
	bShapeTrans.setIdentity();
	bShapeTrans.setOrigin(position);
	bShape = new btConvexHullShape((btVector3(0, 0, 0), btVector3(10, 0, 10), btVector3(20, 0, 0), btVector3(10, 5, 0)), 4, 16);	
	btScalar bmass(1.0f);
	btVector3 bLocalInertia(0, 0, 0);
	bShape->calculateLocalInertia(bmass, bLocalInertia);
}
void Objects::SpawnObjectGroup(std::vector<Objects*> b) {
	objectArray = b;
}
void Objects::SetActive() {

	object->setAnisotropicFriction(oShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
	
	//boids.push_back(boid);
	object->activate(true);
}