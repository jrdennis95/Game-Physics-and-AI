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

struct Boids {
	enum class BoidsStats {
		SPEED
	};

	btConvexHullShape* bShape;
	btTransform bShapeTrans;

	Boids::Boids();
	Boids::~Boids();
	Boids::Boids(btRigidBody* boidRigidBody);

	btConvexHullShape* boidShape;
	btTransform boidTransform;
	btRigidBody* boid;

	void Boids::SetPosition(const btVector3 &position);
	void Boids::SetActive();
	btVector3 Boids::Movement(const btVector3 &point);
	bool Boids::BoundaryWidth();
	btVector3 Boids::BoundaryHeight();
	btConvexHullShape* GetShape() { return bShape; };
	btTransform GetTransform() { return bShapeTrans; };
};
