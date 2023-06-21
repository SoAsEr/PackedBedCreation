#include "StaticShapes.h"

std::vector<physx::PxPlane> createCircleApproximation(float rad, float epsilon) {
	int sides = std::ceilf(physx::PxPi / std::acosf(1.0f - (epsilon / rad)));
	std::vector<physx::PxPlane> allPlanes;
	allPlanes.reserve(sides);
	for (int i = 0; i < sides; ++i) {
		float angle = physx::PxTwoPi * static_cast<float>(i) / sides;
		float x = std::cos(angle);
		float z = std::sin(angle);
		allPlanes.emplace_back(-x, 0, -z, rad);
	}
	return allPlanes;
}

std::vector<physx::PxRigidStatic*> createStaticActors(float innerRad, float outerRad, float height, physx::PxMaterial& material, physx::PxPhysics* mPhysics) {
	std::vector<physx::PxRigidStatic*> actors;
	auto planes = createCircleApproximation(outerRad, outerRad * 0.01f);
	actors.reserve(planes.size() + 3);
	for (auto&& plane : planes) {
		physx::PxRigidStatic* thisPlaneActor = mPhysics->createRigidStatic(physx::PxTransformFromPlaneEquation(plane));
		physx::PxRigidActorExt::createExclusiveShape(*thisPlaneActor, physx::PxPlaneGeometry(), material);
		actors.push_back(thisPlaneActor);
	}

	physx::PxRigidStatic* floorActor = mPhysics->createRigidStatic(physx::PxTransformFromPlaneEquation(physx::PxPlane(0, 1, 0, 0)));
	physx::PxRigidActorExt::createExclusiveShape(*floorActor, physx::PxPlaneGeometry(), material);
	actors.push_back(floorActor);

	physx::PxRigidStatic* ceilingActor = mPhysics->createRigidStatic(physx::PxTransformFromPlaneEquation(physx::PxPlane(0, -1, 0, 10.0f * height)));
	physx::PxRigidActorExt::createExclusiveShape(*ceilingActor, physx::PxPlaneGeometry(), material);
	actors.push_back(ceilingActor);

	physx::PxRigidStatic* cylinderActor = mPhysics->createRigidStatic(physx::PxTransform(physx::PxVec3(0, height / 2.0f, 0), physx::PxQuat(physx::PxHalfPi, physx::PxVec3(0, 0, 1))));
	physx::PxRigidActorExt::createExclusiveShape(*cylinderActor,
		physx::PxCapsuleGeometry(innerRad, height / 2.0f), material);
	actors.push_back(cylinderActor);

	return actors;
}