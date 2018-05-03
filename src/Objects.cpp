#include "Objects.h"

std::vector<Objects*> objectsArray;

Objects::Objects()
{
	object = nullptr;
	objectShape = nullptr;
}


Objects::Objects(btRigidBody* o)
{
	object = o;
	objectShape = nullptr;
}
//Destructor for the objects array
Objects::~Objects() 
{
	for (unsigned int i = 0; i < objectsArray.size(); ++i) {
		delete objectsArray[i];
	}
	objectsArray.clear();
	delete object;
	delete objectShape;
}

//Creates the object shape and values
void Objects::SetShape(const btVector3 &position) {
	bShapeTrans = btTransform();
	bShapeTrans.setIdentity();
	bShapeTrans.setOrigin(position);
	objectShape = new btCylinderShape(btVector3(radius, 100, radius));
	btScalar bmass(1.0f);
	btVector3 bLocalInertia(0, 0, 0);
	objectShape->calculateLocalInertia(bmass, bLocalInertia);
}

//Activates the object
void Objects::SetActive() {
	object->setAnisotropicFriction(objectShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
	object->setFriction(0.5);
	object->activate(true);
}