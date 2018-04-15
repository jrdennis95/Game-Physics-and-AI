#include "Boids.h"
#include <vector>

class BoidGroup {
	btVector3 Alignment(Boids *boid, btScalar speed, btScalar force);
	btVector3 Cohesion(Boids *boid);
	btVector3 Separation(Boids *boid, btScalar speed, btScalar force);
public:
	BoidGroup();
	~BoidGroup();
	std::vector<Boids*> boidsArray;
	btScalar speed;
	btScalar force;

	void SpawnBoidGroup(std::vector<Boids*> boidsArray);
	void SpawnAdditional(btRigidBody* boidspawn);
	void UpdateBoidGroup();
};
