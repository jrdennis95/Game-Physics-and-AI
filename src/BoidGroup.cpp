#include "BoidGroup.h"

std::vector<Boids*> boidsArray;

BoidGroup::BoidGroup() {}
BoidGroup::~BoidGroup() 
{
	for (unsigned int i = 0; i < boidsArray.size(); ++i) {
		delete boidsArray[i];
	}
	boidsArray.clear();

	for (unsigned int i = 0; i < objectsArray.size(); ++i) {
		delete objectsArray[i];
	}
	objectsArray.clear();
}
void BoidGroup::SpawnBoidGroup(std::vector<Boids*> b, std::vector<Objects*> o) {
	boidsArray = b;
	objectsArray = o;
}


btVector3 BoidGroup::Separation(Boids* boid, btScalar speed, btScalar force) {
	btVector3 basePosition(0, 0, 0);
	int neighbours = 0;
	int maxDetectionArea = 40;

	std::vector<btVector3> combined;
	for (unsigned int i = 0; i < boidsArray.size(); i++) {
		if (boidsArray[i]->boid != boid->boid) {
			combined.push_back(boidsArray[i]->boid->getCenterOfMassPosition());
		}
	}
	for (unsigned int i = 0; i < objectsArray.size(); i++) {
		combined.push_back(objectsArray[i]->object->getCenterOfMassPosition());
	}
	for (unsigned int i = 0; i<combined.size(); i++){

			btScalar dist = boid->boid->getCenterOfMassPosition().distance(combined[i]);
			if (dist > 0 && dist < maxDetectionArea) {
				basePosition += btVector3(btScalar(boid->boid->getCenterOfMassPosition().getX() - combined[i].getX()), btScalar(boid->boid->getCenterOfMassPosition().getY() - combined[i].getY()), btScalar(boid->boid->getCenterOfMassPosition().getZ() - combined[i].getZ())).safeNormalize() /dist;
				neighbours++;

		}
	}

	if (neighbours == 0) {
		return basePosition;
	}

	basePosition /= btScalar(neighbours);
	basePosition.safeNormalize();
	basePosition *= speed;

	btVector3 alignvalue = basePosition - boid->boid->getLinearVelocity();
	alignvalue = alignvalue.safeNormalize() * force;
	return alignvalue;
}

btVector3 BoidGroup::Alignment(Boids* boid, btScalar speed, btScalar force) {
	btVector3 baseVel = btVector3(0, 0, 0);
	int neighbours = 0;
	int maxDetectionArea = 60;

	for (unsigned int i = 0; i<boidsArray.size(); i++) {
		btRigidBody* neighbour = boidsArray[i]->boid;

		if (neighbour != boid->boid) {
			btScalar dist = boid->boid->getCenterOfMassPosition().distance(neighbour->getCenterOfMassPosition());

			if (dist > 0 && dist < maxDetectionArea) {
				baseVel = btVector3(baseVel.getX() + neighbour->getLinearVelocity().getX(), baseVel.getY() + neighbour->getLinearVelocity().getY(), baseVel.getZ() + neighbour->getLinearVelocity().getZ());
				neighbours++;
			}
		}
	}

	if (neighbours == 0) {
		return baseVel;
	}

	baseVel /= btScalar(neighbours);
	baseVel.safeNormalize();
	baseVel *= speed;

	btVector3 alignvalue = baseVel - boid->boid->getLinearVelocity();
	alignvalue = alignvalue.safeNormalize() * force;
	return alignvalue;
}

btVector3 BoidGroup::Cohesion(Boids* boid) {
	btVector3 basePosition = btVector3(0, 0, 0);
	int neighbours = 0;
	int maxDetectionArea = 60;
	btAlignedObjectArray<btVector3> posArray;

	for (unsigned int i = 0; i<boidsArray.size(); i++) {
		btRigidBody* neighbour = boidsArray[i]->boid;

		if (neighbour != boid->boid) {
			btScalar dist = boid->boid->getCenterOfMassPosition().distance(neighbour->getCenterOfMassPosition());


			if (dist > 0 && dist < maxDetectionArea) {
				basePosition += btVector3(btScalar(boid->boid->getCenterOfMassPosition().getX() + neighbour->getCenterOfMassPosition().getX()), btScalar(boid->boid->getCenterOfMassPosition().getY() + neighbour->getCenterOfMassPosition().getY()), btScalar(boid->boid->getCenterOfMassPosition().getZ() + neighbour->getCenterOfMassPosition().getZ())).normalize() / dist;
				neighbours++;
			}
		}
	}

	if (neighbours == 0) {
		return basePosition;
	}

	basePosition /= btScalar(neighbours);
	basePosition = boid->Movement(basePosition);
	return basePosition;
}

void BoidGroup::UpdateBoidGroup() {
	for (unsigned int i = 0; i < boidsArray.size(); i++) {

		Boids* body0 = boidsArray[i];
		btRigidBody* bbody0 = body0->boid;
		btVector3 allforces = (Alignment(body0, 20, 15)) + (Separation(body0, 20, 2) * 2) + (Cohesion(body0) * 2);
		if (allforces.getX() == 0 && allforces.getY() == 0 && allforces.getZ() == 0) {
			allforces = btVector3(1, 0, 1);
		}

		btScalar bmass = bbody0->getInvMass();
		btVector3 bvel = bbody0->getLinearVelocity();
		btVector3 bgravity = bbody0->getGravity();

		btTransform btrans(bbody0->getOrientation());
		btVector3 up(0, 1, 0);
		btVector3 btop = btrans * up;
		btVector3 front = btrans * btVector3(1, 0, 0);
		btVector3 bdir = bvel.safeNormalize();
		btVector3 avel = bbody0->getAngularVelocity();
		btVector3 bthrust = 4.f * front;
		btVector3 bdrag = -3 * bvel;
		btVector3 blift = body0->BoundaryHeight() - bgravity;

		bbody0->applyCentralForce((allforces + bthrust + blift + bgravity + bdrag) * bmass);
		bbody0->applyTorque((2.0f * front.cross(bdir) + (-5.0 * avel)) * bmass);
		bbody0->applyTorque((- 0.5 * up) * bmass);
		bbody0->applyTorque((0.5 * btop.cross(up) + (-5.0 * avel)) * bmass);

		//Obstacle avoidance

		for (unsigned int j = 0; j < objectsArray.size(); j++) {
			btScalar intensity1 = 22.5f;
			btVector3 pos1 = btVector3(objectsArray[j]->object->getCenterOfMassPosition().getX(), objectsArray[j]->object->getCenterOfMassPosition().getY(), objectsArray[j]->object->getCenterOfMassPosition().getZ());
			btVector3 pos2 = btVector3(bbody0->getCenterOfMassPosition().getX(), bbody0->getCenterOfMassPosition().getY(), bbody0->getCenterOfMassPosition().getZ());
			btScalar dist = pos2.distance(pos1);
			btVector3 components = pos1 - pos2;
			components.normalize();
			btVector3 force = components * (intensity1 * (1 / dist * dist));
			if (dist <= objectsArray[j]->GetRadius() * 10) {
				bbody0->applyCentralForce((-force * bmass));
			}
		}

		//Boundary avoidance
		
		btScalar intensity2 = 40.f;
		btVector3 pos3 = btVector3(0, bbody0->getCenterOfMassPosition().getY(), 0);
		btVector3 pos4 = btVector3(bbody0->getCenterOfMassPosition().getX(), bbody0->getCenterOfMassPosition().getY(), bbody0->getCenterOfMassPosition().getZ());
		btScalar dist2 = pos4.distance(pos3);
		btVector3 components2 = pos3 - pos4;
		components2.normalize();
		btVector3 force2 = components2 * (intensity2 * (1 / dist2 * dist2));
		if (dist2 > 1000) {
			bbody0->applyCentralForce((force2 * bmass));
		}
		
	}
}