#include "KCRPhysicsWorld.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
//#include "BulletCollision/CollisionShapes/btShapeHull.h"
//#include <stdio.h>

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

//#define USE_PARALLEL_SOLVER 1 //experimental parallel solver
//#define USE_PARALLEL_DISPATCHER 1

#ifdef USE_PARALLEL_DISPATCHER
#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"

#ifdef USE_LIBSPE2
#include "BulletMultiThreaded/SpuLibspe2Support.h"
#elif defined (_WIN32)
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif 

#ifdef USE_PARALLEL_SOLVER
#include "BulletMultiThreaded/btParallelConstraintSolver.h"
#include "BulletMultiThreaded/SequentialThreadSupport.h"

btThreadSupportInterface* createSolverThreadSupport(int maxNumThreads) {
#ifdef _WIN32
	Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("solverThreads",SolverThreadFunc,SolverlsMemoryFunc,maxNumThreads);
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(threadConstructionInfo);
	threadSupport->startSPU();
#endif
	return threadSupport;
}
#endif //USE_PARALLEL_SOLVER

#endif//USE_PARALLEL_DISPATCHER

#define FORCE_ZAXIS_UP 1
#define CUBE_HALF_EXTENTS 1
const int maxProxies = 32766;
const int maxOverlap = 65535;

//static const int HF_Size = 4096;
static const int HF_Size = 16384;
//static const int s_gridSize	= 64;
static const int s_gridSize		= 128;
//static const int s_gridSize		= 64 + 1;  // must be (2^N) + 1
static const float s_gridSpacing	= 1;//5
static const float s_gridHeightScale= 1;//0.2f
//static const float WScale= 1.0;//1.1

static const float car_mass = 250;

KCRPhysicsWorld::KCRPhysicsWorld(void):m_carChassis(0),m_indexVertexArrays(0),m_vertices(0)
{
	m_vehicle = 0;
	m_wheelShape = 0;
	count = 0;
#ifdef FORCE_ZAXIS_UP
	rightIndex = 0; 
	upIndex = 2; 
	forwardIndex = 1;
	wheelDirectionCS0 = btVector3(0,0,-1);
	wheelAxleCS = btVector3(1,0,0);
#else
	rightIndex = 0;
	upIndex = 1;
	forwardIndex = 2;
	wheelDirectionCS0 = btVector3(0,-1,0);
	wheelAxleCS = btVector3(-1,0,0);
#endif

	gEngineForce = 0.f;
	gBreakingForce = 0.f;
	maxEngineForce = 2000.f;//this should be engine/velocity dependent //1000
	maxBreakingForce = 200.f;//100
	defaultBreakingForce = 50.f;//10
	gVehicleSteering = 0.f;
	steeringIncrement = 0.2f;//0.04f
	steeringClamp = 0.5f;//0.3f
	wheelRadius = 0.5f;
	wheelWidth = 0.4f;
	wheelFriction = 1000;//BT_LARGE_FLOAT;
	suspensionStiffness = 20.f;
	suspensionDamping = 2.3f;
	suspensionCompression = 4.4f;
	rollInfluence = 0.1f;//1.0f;
	suspensionRestLength = btScalar(0.6);
	worldDepth = 0;
	m_rawHeightfieldData = new float[HF_Size];
}

KCRPhysicsWorld::~KCRPhysicsWorld(void) { //cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++) {
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	delete m_indexVertexArrays;
	delete m_vertices;
	delete m_dynamicsWorld;
	delete m_vehicleRayCaster;
	delete m_vehicle;
	delete m_wheelShape;
	delete m_constraintSolver;
#ifdef USE_PARALLEL_DISPATCHER
	if (m_threadSupportSolver) {
		delete m_threadSupportSolver;
	}
#endif
	delete m_overlappingPairCache; //delete broadphase
	delete m_dispatcher; //delete dispatcher
#ifdef USE_PARALLEL_DISPATCHER
	if (m_threadSupportCollision) {
		delete m_threadSupportCollision;
	}
#endif
	delete m_collisionConfiguration;

}

void KCRPhysicsWorld::initPhysics() {

#ifdef USE_PARALLEL_DISPATCHER
	m_threadSupportSolver = 0;
	m_threadSupportCollision = 0;
#endif

#ifdef FORCE_ZAXIS_UP
	m_forwardAxis = 1;
#endif

	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	m_collisionShapes.push_back(groundShape);

	btDefaultCollisionConstructionInfo cci;
	cci.m_defaultMaxPersistentManifoldPoolSize = maxProxies;
	m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);

#ifdef USE_PARALLEL_DISPATCHER
	int maxNumOutstandingTasks = 2;

#ifdef USE_WIN32_THREADING

m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#endif
	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
#else
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	
#ifdef USE_PARALLEL_SOLVER
	m_threadSupportSolver = createSolverThreadSupport(maxNumOutstandingTasks);
	m_constraintSolver = new btParallelConstraintSolver(m_threadSupportSolver);
	//this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
	m_dispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);
#else
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	//btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	//m_constraintSolver = solver;
#endif //USE_PARALLEL_SOLVER

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);

#ifdef USE_PARALLEL_SOLVER
	m_dynamicsWorld->getSimulationIslandManager()->setSplitIslands(false);
	m_dynamicsWorld->getSolverInfo().m_numIterations = 4;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
#endif

#ifdef FORCE_ZAXIS_UP
	m_dynamicsWorld->setGravity(btVector3(0,0,-10));
#else
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
#endif 

	btTransform tr;
	tr.setIdentity();

	if (USE_TRIMESH_GROUND) {
		int i;
		const float TRIANGLE_SIZE=20.f;
		//create a triangle-mesh ground
		int vertStride = sizeof(btVector3);
		int indexStride = 3*sizeof(int);

		const int NUM_VERTS_X = 20;
		const int NUM_VERTS_Y = 20;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

		m_vertices = new btVector3[totalVerts];
		int*	gIndices = new int[totalTriangles*3];
		for ( i=0;i<NUM_VERTS_X;i++) {
			for (int j=0;j<NUM_VERTS_Y;j++) {
				float wl = .2f;
				//height set to zero, but can also use curved landscape, just uncomment out the code
				float height = 0.f;//20.f*sinf(float(i)*wl)*cosf(float(j)*wl);

	#ifdef FORCE_ZAXIS_UP
				m_vertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE,height);
	#else
				m_vertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,height,(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
	#endif
			}
		}
		int index=0;
		for ( i=0;i<NUM_VERTS_X-1;i++) {
			for (int j=0;j<NUM_VERTS_Y-1;j++) {
				gIndices[index++] = j*NUM_VERTS_X+i;
				gIndices[index++] = j*NUM_VERTS_X+i+1;
				gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

				gIndices[index++] = j*NUM_VERTS_X+i;
				gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
				gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			}
		}
		
		m_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles, gIndices,indexStride,totalVerts,(btScalar*) &m_vertices[0].x(),vertStride);
		bool useQuantizedAabbCompression = true;
		groundShape = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression);
	} else { //testing btHeightfieldTerrainShape
		bool flipQuadEdges=false;
		PHY_ScalarType	m_type(PHY_FLOAT);

		btHeightfieldTerrainShape * heightfieldShape = new btHeightfieldTerrainShape(s_gridSize, s_gridSize,
														m_rawHeightfieldData,s_gridHeightScale,m_minHeight, m_maxHeight,
														upIndex, m_type, flipQuadEdges);
		groundShape = heightfieldShape;	
		btVector3 localScaling(1,1,1);
		localScaling[upIndex]=1.f;
		groundShape->setLocalScaling(localScaling);
	} //USE_TRIMESH_GROUND

	tr.setOrigin(btVector3(0,0,0));
	m_collisionShapes.push_back(groundShape);
	groundMotionState = new btDefaultMotionState(tr);
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
    groundRigidBody = new btRigidBody(groundRigidBodyCI);
	groundRigidBody->setFriction(btScalar(0.8));
    m_dynamicsWorld->addRigidBody(groundRigidBody);

#ifdef FORCE_ZAXIS_UP
	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,2.f, 0.5f));
	m_collisionShapes.push_back(chassisShape);
	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);

	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(0,0,1.3));//localTrans effectively shifts the center of mass with respect to the chassis
#else
	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
	m_collisionShapes.push_back(chassisShape);
	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);

	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(0,1,0));//localTrans effectively shifts the center of mass with respect to the chassis
#endif

	compound->addChildShape(localTrans,chassisShape);
	
#ifdef FORCE_ZAXIS_UP
	chassisMotionState =  new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
#else
	chassisMotionState =  new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
#endif
	btVector3 localInertia(0, 0, 0);
	compound->calculateLocalInertia(car_mass,localInertia);
	btRigidBody::btRigidBodyConstructionInfo sphereRigidBody1CI(car_mass,chassisMotionState,compound,localInertia);
	m_carChassis = new btRigidBody(sphereRigidBody1CI);

	m_dynamicsWorld->addRigidBody(m_carChassis);

	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));

	/// create vehicle
	{	
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);
		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);
		m_dynamicsWorld->addVehicle(m_vehicle);
		float connectionHeight = 1.2f;
		bool isFrontWheel=true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

#ifdef FORCE_ZAXIS_UP
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif //FORCE_ZAXIS_UP
		isFrontWheel = false;
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		for (int i=0;i<m_vehicle->getNumWheels();i++) {
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}
	resetScene();
	//PointCloud2ConvexHullBody();
	m_clock.reset();
}


void KCRPhysicsWorld::Update() {
	setCarMovement();
	for (int i=0; i < m_vehicle->getNumWheels(); i++) {
		m_vehicle->updateWheelTransform(i,true); 
	}
	m_dynamicsWorld->stepSimulation(1/20.f,5); //60s
	//float ms = getDeltaTimeMicroseconds();
	//m_dynamicsWorld->stepSimulation(ms / 1000000.f);

}

void KCRPhysicsWorld::setCarMovement() {
	//Drive front wheels
	m_vehicle->setSteeringValue(gVehicleSteering,0);
	m_vehicle->setBrake(gBreakingForce,0);
	m_vehicle->setSteeringValue(gVehicleSteering,1);
	m_vehicle->setBrake(gBreakingForce,1);

	//Drive rear wheels
	m_vehicle->applyEngineForce(gEngineForce,2);
	m_vehicle->setBrake(gBreakingForce,2);
	m_vehicle->applyEngineForce(gEngineForce,3);
	m_vehicle->setBrake(gBreakingForce,3);
	//if(gBreakingForce < -defaultBreakingForce)
	//	gBreakingForce = -defaultBreakingForce;
}

btTransform KCRPhysicsWorld::getCarPose() {
	btTransform trans;
	m_carChassis->getMotionState()->getWorldTransform(trans);
	return trans;
}

btScalar KCRPhysicsWorld::getDeltaTimeMicroseconds() {
	btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
	m_clock.reset();
	return dt;
}

void KCRPhysicsWorld::resetEngineForce() {
	gEngineForce = 0.f;
	gBreakingForce = defaultBreakingForce; 
	//printf("reset engine force\n");
}

void KCRPhysicsWorld::accelerateEngine() {
	gEngineForce = maxEngineForce;
	gBreakingForce = 0.f;
	//printf("accelerate \n");
}

void KCRPhysicsWorld::decelerateEngine() {
	gEngineForce = -maxEngineForce;
	gBreakingForce = 0.f;
	//printf("decelerate \n");
}

void KCRPhysicsWorld::turnReset() {
	gVehicleSteering = 0;
}

void KCRPhysicsWorld::turnEngineLeft() {
	gVehicleSteering += steeringIncrement;
	if (gVehicleSteering > steeringClamp)	
		gVehicleSteering = steeringClamp;
	//printf("turn left \n");
}

void KCRPhysicsWorld::turnEngineRight() {
	gVehicleSteering -= steeringIncrement;
	if (gVehicleSteering < -steeringClamp)
		gVehicleSteering = -steeringClamp;
	//printf("turn right \n");
}

void KCRPhysicsWorld::resetScene() {
	gVehicleSteering = 0.f;
	gBreakingForce = defaultBreakingForce;
	gEngineForce = 0.f;

	//m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,5)));
	m_carChassis->setLinearVelocity(btVector3(0,0,0));
	m_carChassis->setAngularVelocity(btVector3(0,0,0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i,true);
		}
	}
}

void KCRPhysicsWorld::setWorldDepth(float w) {
	worldDepth = w;
}

void KCRPhysicsWorld::PointCloud2ConvexHullBody() {

	convexShape = new btConvexHullShape();
#ifdef FORCE_ZAXIS_UP
	convexShape->addPoint(btVector3(0, 0, 0));
	convexShape->addPoint(btVector3(-10, 0, 0));
	convexShape->addPoint(btVector3(-10, 0, 5));
	convexShape->addPoint(btVector3(-20, 0, 0));
	convexShape->addPoint(btVector3(-20, 0, 5));
	convexShape->addPoint(btVector3(0, 30, 0));
	convexShape->addPoint(btVector3(-10, 30, 0));
	convexShape->addPoint(btVector3(-10, 30, 5));
	convexShape->addPoint(btVector3(-20, 30, 0));
	convexShape->addPoint(btVector3(-20, 30, 5));
#else
	convexShape->addPoint(btVector3(0, 0, 0));
	convexShape->addPoint(btVector3(-10, 0, 0));
	convexShape->addPoint(btVector3(-10, 5, 0));
	convexShape->addPoint(btVector3(-20, 0, 0));
	convexShape->addPoint(btVector3(-20, 5, 0));
	convexShape->addPoint(btVector3(0, 0, 30));
	convexShape->addPoint(btVector3(-10, 0, 30));
	convexShape->addPoint(btVector3(-10, 5, 30));
	convexShape->addPoint(btVector3(-20, 0, 30));
	convexShape->addPoint(btVector3(-20, 5, 30));
#endif
	float convexMass = 200;
	btTransform localTrans;
	localTrans.setIdentity();
#ifdef FORCE_ZAXIS_UP
	//localTrans.setOrigin(btVector3(0,0,-4.45));
	localTrans.setOrigin(btVector3(-10,-10,0.1));
#else
	localTrans.setOrigin(btVector3(0,-4.45,0));
#endif
	btDefaultMotionState *convexMotionState = new btDefaultMotionState(localTrans);
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	convexShape->calculateLocalInertia(convexMass, localInertia);

    btRigidBody::btRigidBodyConstructionInfo convexRigidBodyCI(0,convexMotionState,convexShape,btVector3(0,0,0));
    convexBody = new btRigidBody(convexRigidBodyCI);
	m_dynamicsWorld->addRigidBody(convexBody);

}

btConvexHullShape* KCRPhysicsWorld::getConvexShape() {
	return convexShape;
}

btTransform KCRPhysicsWorld::getConvexTransform() {
	btTransform trans;
	convexBody->getMotionState()->getWorldTransform(trans);
	return trans;
}

float KCRPhysicsWorld::getGridHeight(float* grid, int i, int j) {
	btAssert(grid);
	btAssert(i >= 0 && i < s_gridSize);
	btAssert(j >= 0 && j < s_gridSize);

	int bpe = sizeof(float);
	btAssert(bpe > 0 && "bad bytes per element");
	int idx = (j * s_gridSize) + i;
	long offset = ((long) bpe) * idx;
	float *p = grid + offset;

	return *p;
}


float* KCRPhysicsWorld::getHeightfield() {
	return m_rawHeightfieldData;
}

int KCRPhysicsWorld::getGridSize() {
	return s_gridSize;
}

btTransform KCRPhysicsWorld::getHeightFieldTransform() {
	btTransform trans;
	groundRigidBody->getMotionState()->getWorldTransform(trans);
	return trans;
}

void KCRPhysicsWorld::setHeightField(float* hf) {
	for (int i = 0; i < HF_Size; i++) {
		if(m_rawHeightfieldData[i] != hf[i])
			m_rawHeightfieldData[i] = hf[i];
	}
}

void KCRPhysicsWorld::setMinHeight(float min) {
	m_minHeight = btScalar(min);
}

void KCRPhysicsWorld::setMaxHeight(float max) {
	m_maxHeight = btScalar(max);
}

void KCRPhysicsWorld::setWorldScale(float scale) {
	wscale = scale;
}