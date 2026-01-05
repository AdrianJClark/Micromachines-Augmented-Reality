
#ifndef KCR_PHYS_WORLD_H
#define KCR_PHYS_WORLD_H
//#pragma once

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

const bool USE_TRIMESH_GROUND = false;

class KCRPhysicsWorld {
	public:

		class btThreadSupportInterface*		m_threadSupportCollision;
		class btThreadSupportInterface*		m_threadSupportSolver;

		btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
		btBroadphaseInterface*	m_overlappingPairCache;
		btCollisionDispatcher*	m_dispatcher;
		btConstraintSolver*	m_constraintSolver;
		btDefaultCollisionConfiguration* m_collisionConfiguration;
		btTriangleIndexVertexArray*	m_indexVertexArrays;
		btDiscreteDynamicsWorld* m_dynamicsWorld;

		btDefaultMotionState* groundMotionState, *chassisMotionState;
		btRigidBody* groundRigidBody, *m_carChassis;
		btConvexHullShape *convexShape;
		btRigidBody *convexBody;

		btVector3*	m_vertices;
		btRaycastVehicle::btVehicleTuning	m_tuning;
		btVehicleRaycaster*	m_vehicleRayCaster;
		btRaycastVehicle*	m_vehicle;
		btCollisionShape*	m_wheelShape;
		btClock m_clock;

		KCRPhysicsWorld(void);
		virtual ~KCRPhysicsWorld(void);
		virtual void Update();

		void initPhysics();

		void setCarMovement();
		btTransform getCarPose();
		float* Quaternion2RotMat3x3(btScalar x, btScalar y, btScalar z, btScalar w);
		btScalar virtual getDeltaTimeMicroseconds();
		void resetEngineForce();
		void accelerateEngine();
		void decelerateEngine();
		void turnReset();
		void turnEngineLeft();
		void turnEngineRight();
		void resetScene();
		void setWorldDepth(float w);
		void PointCloud2ConvexHullBody();
		btConvexHullShape* getConvexShape();
		btTransform getConvexTransform();
		//
		float getGridHeight(float* grid, int i, int j);
		float* getHeightfield();
		int getGridSize();
		btTransform getHeightFieldTransform();
		void setHeightField(float* hf);
		void setMinHeight(float min);
		void setMaxHeight(float max);
		void setWorldScale(float scale);
		//
	private:
		int count;
		int rightIndex;
		int upIndex;
		int forwardIndex;
		btVector3 wheelDirectionCS0;
		btVector3 wheelAxleCS;
		float	m_forwardAxis;
		float	gEngineForce;
		float	gBreakingForce;
		float	maxEngineForce;
		float	maxBreakingForce;
		float	defaultBreakingForce;
		float	gVehicleSteering;
		float	steeringIncrement;
		float	steeringClamp;
		float	wheelRadius;
		float	wheelWidth;
		float	wheelFriction;
		float	suspensionStiffness;
		float	suspensionDamping;
		float	suspensionCompression;
		float	rollInfluence;
		float	worldDepth;
		float	wscale;
		btScalar suspensionRestLength;
		//
		float*	m_rawHeightfieldData;
		btScalar	m_minHeight;
		btScalar	m_maxHeight;
};

#endif //KCR_PHYS_WORLD_H