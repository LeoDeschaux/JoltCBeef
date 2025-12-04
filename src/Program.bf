using System;
using System.IO;
using System.Collections;
using System.Diagnostics;

using static Jolt.Jolt;
namespace Jolt;

static class Program
{
	/*
	static void TraceImpl(char* message)
	{
		// Print to the TTY
		std::cout << message << std::endl;
	}
	*/

	struct Layers
	{
		public static JPH_ObjectLayer NON_MOVING = 0;
		public static JPH_ObjectLayer MOVING = 1;
		public static JPH_ObjectLayer NUM_LAYERS = 2;
	};

	struct BroadPhaseLayers
	{
		public static JPH_BroadPhaseLayer NON_MOVING = 0;
		public static JPH_BroadPhaseLayer MOVING = 1;
		public static int NUM_LAYERS = 2;
	};

	public static int Main(String[] args)
	{
		if (!JPH_Init())
			return 1;

		//JPH_SetTraceHandler(TraceImpl);
		//JPH_SetAssertFailureHandler(JPH_AssertFailureFunc handler);

		JPH_JobSystem* jobSystem = JPH_JobSystemThreadPool_Create(null);

		// We use only 2 layers: one for non-moving objects and one for moving objects
		JPH_ObjectLayerPairFilter* objectLayerPairFilterTable = JPH_ObjectLayerPairFilterTable_Create(2);
		JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilterTable, Layers.NON_MOVING, Layers.MOVING);
		JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilterTable, Layers.MOVING, Layers.NON_MOVING);

		// We use a 1-to-1 mapping between object layers and broadphase layers
		JPH_BroadPhaseLayerInterface* broadPhaseLayerInterfaceTable = JPH_BroadPhaseLayerInterfaceTable_Create(2, 2);
		JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterfaceTable, Layers.NON_MOVING, BroadPhaseLayers.NON_MOVING);
		JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterfaceTable, Layers.MOVING, BroadPhaseLayers.MOVING);

		JPH_ObjectVsBroadPhaseLayerFilter* objectVsBroadPhaseLayerFilter = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(broadPhaseLayerInterfaceTable, 2, objectLayerPairFilterTable, 2);

		JPH_PhysicsSystemSettings settings = .();
		settings.maxBodies = 65536;
		settings.numBodyMutexes = 0;
		settings.maxBodyPairs = 65536;
		settings.maxContactConstraints = 65536;
		settings.broadPhaseLayerInterface = broadPhaseLayerInterfaceTable;
		settings.objectLayerPairFilter = objectLayerPairFilterTable;
		settings.objectVsBroadPhaseLayerFilter = objectVsBroadPhaseLayerFilter;
		JPH_PhysicsSystem* system = JPH_PhysicsSystem_Create(&settings);
		JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(system);

		JPH_BodyID floorId = .();
		{
			// Next we can create a rigid body to serve as the floor, we make a large box
			// Create the settings for the collision volume (the shape). 
			// Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
			JPH_Vec3 boxHalfExtents = .(100.0f, 1.0f, 100.0f);
			JPH_BoxShape* floorShape = JPH_BoxShape_Create(&boxHalfExtents, JPH_DEFAULT_CONVEX_RADIUS);

			JPH_Vec3 floorPosition = .(0.0f, -1.0f, 0.0f);
			JPH_BodyCreationSettings* floorSettings = JPH_BodyCreationSettings_Create3(
				(JPH_Shape*)floorShape,
				&floorPosition,
				null, // Identity, 
				JPH_MotionType.Static,
				Layers.NON_MOVING);

			// Create the actual rigid body
			floorId = JPH_BodyInterface_CreateAndAddBody(bodyInterface, floorSettings,JPH_Activation.DontActivate);
			JPH_BodyCreationSettings_Destroy(floorSettings);
		}

		// Sphere
		JPH_BodyID sphereId = .();
		{
			JPH_SphereShape* sphereShape = JPH_SphereShape_Create(50.0f);
			JPH_Vec3 spherePosition = .(0.0f, 2.0f, 0.0f);
			JPH_BodyCreationSettings* sphereSettings = JPH_BodyCreationSettings_Create3(
				(JPH_Shape*)sphereShape,
				&spherePosition,
				null, // Identity, 
				JPH_MotionType.Dynamic,
				Layers.MOVING);

			sphereId = JPH_BodyInterface_CreateAndAddBody(bodyInterface, sphereSettings, JPH_Activation.Activate);
			JPH_BodyCreationSettings_Destroy(sphereSettings);
		}

		// Now you can interact with the dynamic body, in this case we're going to give it a velocity.
		// (note that if we had used CreateBody then we could have set the velocity straight on the body before adding it to the physics system)
		JPH_Vec3 sphereLinearVelocity = .(0.0f, -5.0f, 0.0f);
		JPH_BodyInterface_SetLinearVelocity(bodyInterface, sphereId, &sphereLinearVelocity);

		JPH_SixDOFConstraintSettings jointSettings;
		JPH_SixDOFConstraintSettings_Init(&jointSettings);

		// We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
		const float cDeltaTime = 1.0f / 60.0f;

		// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
		// You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
		// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
		JPH_PhysicsSystem_OptimizeBroadPhase(system);

		// Now we're ready to simulate the body, keep simulating until it goes to sleep
		int step = 0;
		while (JPH_BodyInterface_IsActive(bodyInterface, sphereId))
		{
			// Next step
			++step;

			// Output current position and velocity of the sphere
			JPH_RVec3* position = new .(0,0,0);
			JPH_Vec3* velocity= new .(0,0,0);

			JPH_BodyInterface_GetCenterOfMassPosition(bodyInterface, sphereId, position);
			JPH_BodyInterface_GetLinearVelocity(bodyInterface, sphereId, velocity);
			Console.WriteLine(scope $"Step {step}: Position = {position.x},  {position.y}, {position.z}, Velocity = {velocity.x}, {velocity.y}, {velocity.z}");

			// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
			const int cCollisionSteps = 1;

			// Step the world
			JPH_PhysicsSystem_Update(system, cDeltaTime, cCollisionSteps, jobSystem);

			delete position;
			delete velocity;
		}

		// Remove the destroy sphere from the physics system. Note that the sphere itself keeps all of its state and can be re-added at any time.
		JPH_BodyInterface_RemoveAndDestroyBody(bodyInterface, sphereId);

		// Remove and destroy the floor
		JPH_BodyInterface_RemoveAndDestroyBody(bodyInterface, floorId);

		JPH_JobSystem_Destroy(jobSystem);

		JPH_PhysicsSystem_Destroy(system);
		JPH_Shutdown();
		return 0;
	}
}