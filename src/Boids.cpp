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
	//boids.push_back(boid);
	boid->activate(true);
}

btVector3 Boids::Avoid(std::vector<Objects*> &objects) {

	btVector3 bposition = btVector3(boid->getCenterOfMassPosition().getX(), 0.0, boid->getCenterOfMassPosition().getZ());
	btVector3 final(0, 0, 0);
	for (int i = 0; i < objects.size(); i++) {
		Objects* object = objects[i];
		btRigidBody* objectRB = object->object;
		btVector3 a = boid->getCenterOfMassPosition();
		btVector3 b = boid->getLinearVelocity();
			//if (p.length() < v && b.length() < objects[i]->GetRadius() + 20) {
		btScalar radius = object->GetRadius() * 40;
		btVector3 origin = objectRB->getCenterOfMassPosition();
		btVector3 boidpos = boid->getCenterOfMassPosition();
		btVector3 pos1 = btVector3(origin.getX(), 0, origin.getZ());
		btVector3 pos2 = btVector3(boidpos.getX(), 0, boidpos.getZ());
		btScalar dist = pos2.distance(pos1);

			if(dist <= radius){
				btVector3 target(boid->getCenterOfMassPosition());
				btVector3 vector1 = target - origin;
				btVector3 top(origin.getX(), 0, origin.getZ() + 1);
				float angle = atan2(vector1.getX(), vector1.getZ()) - atan2(top.getX(), top.getZ());
				angle = angle * (180 / SIMD_PI);
				if (angle < 0) angle += 360;
				btScalar turn = -0.3f;
				if (angle > 0 && angle <= 90) {
					if (angle > 0 && angle < 45) {
						final = btVector3(0, -(turn), 0);
					}
					else {
						final = btVector3(0, turn, 0);
					}
				}
				else if (angle > 90 && angle <= 180) {
					if (angle > 90 && angle < 135) {
						final = btVector3(0, -(turn), 0);
					}
					else {
						final = btVector3(0, turn, 0);
					}
				}
				else if (angle > 180 && angle <= 270) {
					if (angle > 180 && angle < 225) {
						final = btVector3(0, -(turn), 0);
					}
					else{
						final = btVector3(0, turn, 0);
					}
				}
				else if (angle > 270 && angle <= 360) {
					if (angle > 225 && angle < 360) {
						final = btVector3(0, -(turn), 0);
					}
					else{
						final = btVector3(0, turn, 0);
					}
				}
				else
					final = btVector3(0, turn, 0);
			} else 
				final = btVector3(0, 0, 0);
	}
	return final;
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
	if (btScalar(100) - boid->getCenterOfMassPosition().getY() < 30) {
		return btVector3(0, -3, 0);
	}
	else if (boid->getCenterOfMassPosition().getY() < 30) {
		return btVector3(0, 3, 0);
	}
	else {
		return btVector3(0, 0, 0);
	}
}
