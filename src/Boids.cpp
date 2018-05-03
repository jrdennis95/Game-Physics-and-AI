#include "Boids.h"

Boids::Boids() {

	boid = nullptr;
	boidShape = nullptr;
}

Boids::Boids(btRigidBody* boidRigidBody) {

	boid = boidRigidBody;
	boidShape = nullptr;
}

Boids::~Boids() {

	delete boid;
	delete boidShape;
}

void Boids::SetShape(const btVector3 &position) {
	bShapeTrans = btTransform();
	bShapeTrans.setIdentity();
	bShapeTrans.setOrigin(position);
	btScalar scale = 1.f;
	bShape = new btConvexHullShape((btVector3(0, 0, 0), btVector3(10*scale, 0, 10 * scale), btVector3(20 * scale, 0, 0), btVector3(10 * scale, 5 * scale, 0)), 4, 16);

	btScalar bmass(1.0f);
	btVector3 bLocalInertia(0,0,0);
	bShape->calculateLocalInertia(bmass, bLocalInertia);
}

void Boids::SetActive() {
	
	boid->setAnisotropicFriction(bShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
	boid->setFriction(0.5);
	boid->setLinearVelocity(btVector3(1, 0, 0));
	boid->activate(true);
}

btVector3 Boids::Movement(const btVector3 &point) {
	btVector3 endPos = (point - boid->getCenterOfMassPosition()).normalize() * btScalar(20);
	btVector3 endForce = (endPos - boid->getLinearVelocity()).normalize() * btScalar(2);
	return endForce;
}

btVector3 Boids::BoundaryHeight() {
	if (boid->getCenterOfMassPosition().getY() > 70) {
		return btVector3(0, -6, 0);
	}
	else if (boid->getCenterOfMassPosition().getY() < 30) {
		return btVector3(0, 3, 0);
	}
	else {
		return btVector3(0, 0, 0);
	}
}
