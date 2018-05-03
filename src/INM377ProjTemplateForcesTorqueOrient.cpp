/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#define CUBE_HALF_EXTENTS 1

#define EXTRA_HEIGHT 1.f

#include "INM377ProjTemplateForcesTorqueOrient.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"

#if 0
extern btAlignedObjectArray<btVector3> debugContacts;
extern btAlignedObjectArray<btVector3> debugNormals;
#endif 

static GLDebugDrawer	sDebugDrawer;


void INM377ProjTemplateTorqueOrient::BoidGroupInit() {
	std::vector<Boids*> boids;
	int x = 50;
	int y = 14;
	std::vector<Objects*> objects;
	boids.reserve(x);
	objects.reserve(y);

	//Push boids and objects into their arrays
	for (int i = 0; i < x; i++)
		boids.push_back(new Boids());

	for (int i = 0; i < y; i++)
		objects.push_back(new Objects());

	//Spawns boids and objects
	group.SpawnBoidGroup(boids, objects);
}

//Create boids
void INM377ProjTemplateTorqueOrient::CreateBoids() {
	for (int i = 0; i < 50; i++)
		AddBoids(i, btVector3((rand() % (400 + 1 - 0) + 0)-800, 2, (rand() % (400 + 1 - 0) + 0)));
}

//Create objects
void INM377ProjTemplateTorqueOrient::CreateObjects() {
	std::vector<btVector3> pos = {
		btVector3(10, 1, 10), btVector3(200, 1, 500), btVector3(200, 1, -500), btVector3(250, 1, 850), btVector3(300, 1, 600), btVector3(250, 1, 200), btVector3(250, 1, -200), btVector3(200, 1, 800), btVector3(200, 1, -800), btVector3(-300, 1, -350), btVector3(-700, 1, -500), btVector3(700, 1, -500), btVector3(-900, 1, -900), btVector3(900, 1, -900),
	};
	for (int i = 0; i < 14; i++) {
		AddObjects(i, pos[i]);
	}
}
void INM377ProjTemplateTorqueOrient::AddBoids(int &i, btVector3 &pos) {

	group.boidsArray[i]->SetShape(pos);
	m_collisionShapes.push_back(group.boidsArray[i]->GetShape());
	group.boidsArray[i]->boid = localCreateRigidBody(1.0f, group.boidsArray[i]->GetTransform(), group.boidsArray[i]->GetShape());
	group.boidsArray[i]->SetActive();
}

void INM377ProjTemplateTorqueOrient::AddObjects(int &i, btVector3 &pos) {
	group.objectsArray[i]->SetShape(pos);
	m_collisionShapes.push_back(group.objectsArray[i]->GetShape());
	group.objectsArray[i]->object = localCreateRigidBody(100000, group.objectsArray[i]->GetTransform(), group.objectsArray[i]->GetShape());
	group.objectsArray[i]->SetActive();
}


INM377ProjTemplateTorqueOrient::INM377ProjTemplateTorqueOrient()
:m_ccdMode(USE_CCD)
{
	BoidGroupInit();
	setDebugMode(btIDebugDraw::DBG_DrawText+btIDebugDraw::DBG_NoHelpText);
	setCameraDistance(btScalar(40.));
}

void INM377ProjTemplateTorqueOrient::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	//float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1./60.,0);//ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

//	displayText();
#if 0
	for (int i=0;i<debugContacts.size();i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i],debugNormals[i],0,0,btVector3(1,0,0));
	}
#endif

	glFlush();

	swapBuffers();

}

void INM377ProjTemplateTorqueOrient::displayText()
{
	int lineWidth=440;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_DrawText)!=0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		glRasterPos3f(xStart, yStart, 0);
		switch (m_ccdMode)
		{
		case USE_CCD:
			{
				sprintf_s(buf,"Predictive contacts and motion clamping");
				break;
			}
		case USE_NO_CCD:
			{
				sprintf_s(buf,"CCD handling disabled");
				break;
			}
		default:
			{
				sprintf_s(buf,"unknown CCD setting");
			};
		};

		GLDebugDrawString(xStart,20,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf_s(buf,"Press 'p' to change CCD mode");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf_s(buf,"Press '.' or right mouse to shoot bullets");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf_s(buf,"space to restart, h(elp), t(ext), w(ire)");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		
		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}	

}

void INM377ProjTemplateTorqueOrient::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	displayText();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->debugDrawWorld();
	}
#if 0
	for (int i=0;i<debugContacts.size();i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i],debugNormals[i],0,0,btVector3(1,0,0));
	}
#endif

	glFlush();
	swapBuffers();
}


void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
		world->clearForces();
		static_cast<INM377ProjTemplateTorqueOrient *>(world->getWorldUserInfo())->group.UpdateBoidGroup();
}

void	INM377ProjTemplateTorqueOrient::initPhysics()
{
	setTexturing(true);
	setShadows(false);

	setCameraDistance(350.f);

	// init world
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	btDiscreteDynamicsWorld* wp = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);
	//	wp->getSolverInfo().m_numIterations = 20; // default is 10
	m_dynamicsWorld = wp;
	m_dynamicsWorld->setInternalTickCallback(MyTickCallback, static_cast<void *>(this), true);
	

	///create a few basic rigid bodies
	CreateGround();
	CreateBoids();
	CreateObjects();

}

void INM377ProjTemplateTorqueOrient::CreateGround() {
	btBoxShape* box = new btBoxShape(btVector3(btScalar(1100.0f), btScalar(1.0f), btScalar(1100.0f)));
	//	box->initializePolyhedralFeatures();
	btCollisionShape* groundShape = box;

	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);
	//m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
	m_collisionShapes.push_back(new btBoxShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS)));

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setFriction(0.5);

		body->setRollingFriction(0.3);
		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
}

void	INM377ProjTemplateTorqueOrient::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void INM377ProjTemplateTorqueOrient::keyboardCallback(unsigned char key, int x, int y)
{
	if (key=='p')
	{
		switch (m_ccdMode)
		{
			case USE_CCD:
			{
				m_ccdMode = USE_NO_CCD;
				break;
			}
			case USE_NO_CCD:
			default:
			{
				m_ccdMode = USE_CCD;
			}
		};
		clientResetScene();
	} else
	{
		DemoApplication::keyboardCallback(key,x,y);
	}
}

void	INM377ProjTemplateTorqueOrient::shootBox(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		float mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape ();


		btRigidBody* body = this->localCreateRigidBody(mass, startTransform,m_shootBoxShape);
		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setContactProcessingThreshold(1e30);

		///when using m_ccdMode, disable regular CCD
		if (m_ccdMode==USE_CCD)
		{
			body->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
			body->setCcdSweptSphereRadius(0.4f);
		}
		
	}
}

void	INM377ProjTemplateTorqueOrient::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
//	delete m_solver;
	
//	delete m_broadphase;
	
//	delete m_dispatcher;

//	delete m_collisionConfiguration;

	
}




