#pragma once
#include <PxPhysics.h>
#include <PxPhysicsAPI.h>

#include <vector>
#include <cmath>

std::vector<physx::PxRigidStatic*> createStaticActors(float innerRad, float outerRad, float height, physx::PxMaterial& material, physx::PxPhysics* mPhysics);