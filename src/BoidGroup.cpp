#include "BoidGroup.h"

std::vector<Boids*> boidsArray;
btScalar speed = 20;
btScalar force = 2;
//btVector3 Align(Boids *boid);
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
void BoidGroup::SpawnAdditional(btRigidBody* boidspawn) {
	//boidsArray.push_back(new Boids(boidspawn));
}

btVector3 BoidGroup::Separation(Boids* boid, btScalar speed, btScalar force) {
	btVector3 basePosition(0, 0, 0);
	int neighbours = 0;
	int maxDetectionArea = 20;

	std::vector<btVector3> combined;
	for (unsigned int i = 0; i < boidsArray.size(); i++) {
		if (boidsArray[i]->boid != boid->boid) {
			combined.push_back(boidsArray[i]->boid->getCenterOfMassPosition());
		}
	}
	for (unsigned int i = 0; i < objectsArray.size(); i++) {
		combined.push_back(objectsArray[i]->object->getCenterOfMassPosition()); //maybe wrong
	}
	for (unsigned int i = 0; i<combined.size(); i++){

			btScalar dist = boid->boid->getCenterOfMassPosition().distance(combined[i]);
			if (dist > 0 && dist < maxDetectionArea) {
				btVector3 neighbourDist = combined[i];
				btVector3 temp = btVector3(btScalar(boid->boid->getCenterOfMassPosition().getX() - neighbourDist.getX()), btScalar(boid->boid->getCenterOfMassPosition().getY() - neighbourDist.getY()), btScalar(boid->boid->getCenterOfMassPosition().getZ() - neighbourDist.getZ())).safeNormalize() /dist;
				basePosition += temp;
				neighbours++;

		}
	}

	if (neighbours == 0) {
		return basePosition;
	}

	btScalar neighboursScalar = btScalar(neighbours);
	basePosition /= neighboursScalar;
	basePosition.safeNormalize();
	basePosition *= speed;

	btVector3 alignvalue = basePosition - boid->boid->getLinearVelocity();
	alignvalue = alignvalue.safeNormalize() * force;
	return alignvalue;
}

btVector3 BoidGroup::Alignment(Boids* boid, btScalar speed, btScalar force) {
	btVector3 baseVel = btVector3(0, 0, 0);
	int neighbours = 0;
	int maxDetectionArea = 40;
	int test0 = boidsArray.size();

	for (unsigned int i = 0; i<boidsArray.size(); i++) {
		btRigidBody* neighbour = boidsArray[i]->boid;

		if (neighbour != boid->boid) {
			btScalar dist = boid->boid->getCenterOfMassPosition().distance(neighbour->getCenterOfMassPosition());

			if (dist > 0 && dist < maxDetectionArea) {
				btVector3 neighbourVel = neighbour->getLinearVelocity();
				btScalar x = baseVel.getX() + neighbourVel.getX();
				btScalar y = baseVel.getY() + neighbourVel.getY();
				btScalar z = baseVel.getZ() + neighbourVel.getZ();
				baseVel = btVector3(x, y, z);
				neighbours++;
			}
		}
	}

	if (neighbours == 0) {
		return baseVel;
	}

	btScalar neighboursScalar = btScalar(neighbours);
	baseVel = btVector3(baseVel.getX() / neighboursScalar, baseVel.getY() / neighboursScalar, baseVel.getZ() / neighboursScalar);
	baseVel.safeNormalize();
	baseVel *= speed;

	btVector3 alignvalue = baseVel - boid->boid->getLinearVelocity();
	alignvalue = alignvalue.safeNormalize() * speed;
	return alignvalue;
}

btVector3 BoidGroup::Cohesion(Boids* boid) {
	btVector3 basePosition = btVector3(0, 0, 0);
	int neighbours = 0;
	int maxDetectionArea = 40;
	btAlignedObjectArray<btVector3> posArray;

	for (unsigned int i = 0; i<boidsArray.size(); i++) {
		btRigidBody* neighbour = boidsArray[i]->boid;

		if (neighbour != boid->boid) {
			btScalar dist = boid->boid->getCenterOfMassPosition().distance(neighbour->getCenterOfMassPosition());


			if (dist > 0 && dist < maxDetectionArea) {
				btVector3 neighbourPos = neighbour->getCenterOfMassPosition();
				btVector3 temp = btVector3(btScalar(boid->boid->getCenterOfMassPosition().getX() + neighbourPos.getX()), btScalar(boid->boid->getCenterOfMassPosition().getY() + neighbourPos.getY()), btScalar(boid->boid->getCenterOfMassPosition().getZ() + neighbourPos.getZ())).normalize() / dist;
				basePosition = temp;
				neighbours++;
			}
		}
	}

	if (neighbours == 0) {
		return basePosition;
	}

	btScalar neighboursScalar = btScalar(neighbours);
	basePosition /= neighboursScalar;
	basePosition = boid->Movement(basePosition);
	return basePosition;
}

void BoidGroup::UpdateBoidGroup() {
	for (unsigned int i = 0; i < boidsArray.size(); i++) {

		Boids* body0 = boidsArray[i];
		btRigidBody* bbody0 = body0->boid;
		btVector3 a = Alignment(body0, 20, 2);
			btVector3 b = Separation(body0, 20, 2) * 2;
			btVector3 c = Cohesion(body0);
		btVector3 allforces = Alignment(body0, 20, 2) + Separation(body0, 20, 2) + Cohesion(body0);
		btVector3 all2 = allforces;
		if (allforces.getX() == 0 && allforces.getY() == 0 && allforces.getZ() == 0) {
			allforces = btVector3(1, 0, 1);
		}

		btScalar bmass = bbody0->getInvMass();
		btVector3 bvel = bbody0->getLinearVelocity();
		btVector3 bgravity = bbody0->getGravity();
		//btVector3 bdir = btVector3(0, 0.2, 1);
		btTransform btrans(bbody0->getOrientation());
		btVector3 up(0, 1, 0);
		btVector3 btop = btrans * up;
		btVector3 front = btrans * btVector3(1, 0, 0);
		btVector3 bdir = bvel.safeNormalize();
		btVector3 avel = bbody0->getAngularVelocity();
		btVector3 bthrust = 4.f * front;
		btVector3 bdrag = btScalar(-3) * bvel;
		btVector3 blift = body0->BoundaryHeight() - bgravity;

		bbody0->applyCentralForce((allforces + blift + bgravity + bthrust + bdrag) * bmass/*+ allforces + bgravity + bdrag*/);
		bbody0->applyTorque((2.0f * front.cross(bdir) + (-5.0 * avel)) * bmass);
		bbody0->applyTorque((- 0.5 * up) * bmass);
		bbody0->applyTorque((0.5 * btop.cross(up) + (-5.0 * avel)) * bmass);

		if (body0->BoundaryWidth() == true) {
			bbody0->applyTorque(btVector3(0,2,0) * bmass);
		}

	}
}
