#pragma once
#include "btBulletDynamicsCommon.h"
#include <vector>
#include "Objects.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

struct Boids {

	btConvexHullShape* bShape;
	btTransform bShapeTrans;

	Boids::Boids();
	Boids::~Boids();
	Boids::Boids(btRigidBody* boidRigidBody);

	btConvexHullShape* boidShape;
	btTransform boidTransform;
	btRigidBody* boid;

	void Boids::SetShape(const btVector3 &position);
	void Boids::SetActive();
	btVector3 Boids::Movement(const btVector3 &point);
	btVector3 Boids::BoundaryHeight();
	btConvexHullShape* GetShape() { return bShape; };
	btTransform GetTransform() { return bShapeTrans; };
};
