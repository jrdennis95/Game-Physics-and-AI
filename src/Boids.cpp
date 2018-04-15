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

void Boids::SetPosition(const btVector3 &position) {
	bShapeTrans = btTransform();
	bShapeTrans.setIdentity();
	bShapeTrans.setOrigin(position);
	bShape = new btConvexHullShape((btVector3(0, 0, 0), btVector3(10, 0, 10), btVector3(20, 0, 0), btVector3(10, 5, 0)), 4, 16);
	/*bShape = new btConvexHullShape((btVector3(0, 0, 0), btVector3(10, 0, 10), btVector3(20, 0, 0), btVector3(10, 5, 0)), 4, 16);
	bShape->addPoint(btVector3(2, 0, 0));
	bShape->addPoint(btVector3(0, 0.5f, 0));
	bShape->addPoint(btVector3(0, 0, 1));
	bShape->addPoint(btVector3(0, 0, -1));*/
	//m_collisionShapes.push_back(bShape);
	//btCollisionShape* bshape = m_collisionShapes[3];
	//btVector3 bpos(50 * i, 0, 0);

	btScalar bmass(1.0f);
	btVector3 bLocalInertia(0,0,0);
	bShape->calculateLocalInertia(bmass, bLocalInertia);
}

void Boids::SetActive() {
	
	boid->setAnisotropicFriction(bShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
	boid->setFriction(0.5);
	boid->setLinearVelocity(btVector3(1, 0, 0));
	//boids.push_back(boid);
	boid->activate(true);
}

btVector3 Boids::Movement(const btVector3 &point) {
	btVector3 endPos = (point - boid->getCenterOfMassPosition()).normalize() * btScalar(20);
	btVector3 endForce = (endPos - boid->getLinearVelocity()).normalize() * btScalar(2);
	return endForce;
}

bool Boids::BoundaryWidth() {
	bool boundary = false;
	if (btVector3(0, 0, 0).distance(boid->getCenterOfMassPosition()) > 150) {
		boundary = true;
	}
	return boundary;
}

btVector3 Boids::BoundaryHeight() {
	if (btScalar(130) - boid->getCenterOfMassPosition().getY() < 30) {
		return btVector3(0, -3, 0);
	}
	else if (boid->getCenterOfMassPosition().getY() < 30) {
		return btVector3(0, 3, 0);
	}
	else {
		return btVector3(0, 0, 0);
	}
}
