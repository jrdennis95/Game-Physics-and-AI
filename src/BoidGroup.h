#include "Boids.h"
#include "Objects.h"
#include <vector>


class BoidGroup {
	btVector3 Alignment(Boids *boid, btScalar speed, btScalar force);
	btVector3 Cohesion(Boids *boid);
	btVector3 Separation(Boids *boid, btScalar speed, btScalar force);
public:
	BoidGroup();
	~BoidGroup();
	std::vector<Boids*> boidsArray;
	std::vector<Objects*> objectsArray;

	void SpawnBoidGroup(std::vector<Boids*> b, std::vector<Objects*> o);
	void UpdateBoidGroup();
};
