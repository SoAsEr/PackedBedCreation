#include "StaticShapes.h"

#include <random>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <charconv>
#include <string_view>
#include <array>


#include <PxPhysics.h>
#include <PxPhysicsAPI.h>

int main() {
	physx::PxDefaultAllocator mDefaultAllocatorCallback;
	physx::PxDefaultErrorCallback mDefaultErrorCallback;
	physx::PxDefaultCpuDispatcher* mDispatcher=nullptr;
	physx::PxTolerancesScale mToleranceScale;
	physx::PxFoundation* mFoundation= nullptr;
	physx::PxPhysics* mPhysics= nullptr;
	physx::PxScene* mScene= nullptr;
	physx::PxMaterial* mWallMaterial= nullptr;
	physx::PxMaterial* mBallMaterial= nullptr;
	physx::PxPvd* mPvd= nullptr;

    mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mDefaultAllocatorCallback, mDefaultErrorCallback);
    if (!mFoundation) throw("PxCreateFoundation failed!");
    mPvd = PxCreatePvd(*mFoundation);
    physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    mPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
    //mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(),true, mPvd);
    mToleranceScale.length = 0.6f;        // typical length of an object
    mToleranceScale.speed = 98.1f;         // typical speed of an object, gravity*1s is a reasonable choice
    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, mToleranceScale, true, mPvd);
    //mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, mToleranceScale);

    physx::PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
    sceneDesc.gravity = physx::PxVec3(0.0f, -981.f, 0.0f);
    mDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = mDispatcher;
    sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
    sceneDesc.broadPhaseType = physx::PxBroadPhaseType::eMBP;
    mScene = mPhysics->createScene(sceneDesc);

    float inner = 20.32f/2.f;
    float outer = 24.765f/2.f;
    float height = 30.48f;
    float ballRad = 0.6f/2.f*1.005;


    physx::PxBounds3 bounds(physx::PxVec3(-outer*1.1, -1, -outer*1.1), physx::PxVec3(outer*1.1, 10*height, outer*1.1 ));
    mScene->addBroadPhaseRegion(physx::PxBroadPhaseRegion{ bounds, nullptr });

    physx::PxPvdSceneClient* pvdClient = mScene->getScenePvdClient();
    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }


    // create simulation
    mWallMaterial = mPhysics->createMaterial(0.8f, 0.8f, 0.2f);
    auto actors=createStaticActors(inner, outer, height, *mWallMaterial, mPhysics);
    for (physx::PxRigidStatic* body : actors) {
        mScene->addActor(*body);
    }

    mBallMaterial = mPhysics->createMaterial(0.8f, 0.8f, 0.2f);

    float averageRad = (outer + inner) / 2.f;
    int numPerRing = (physx::PxTwoPi * averageRad) / ballRad/2.25;
    float heightSpacing = ballRad * 2.5;
    int numBalls = 3000;

    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> uni(-ballRad*0.01, ballRad*0.01);
    std::vector<physx::PxRigidDynamic*> ballActors;
    ballActors.reserve(numBalls);
    for (int i = 0; i < numBalls; i++) {
        float angle = (i % numPerRing) * physx::PxTwoPi / numPerRing;
        physx::PxRigidDynamic* ballActor = mPhysics->createRigidDynamic(physx::PxTransform(physx::PxVec3(averageRad *std::cos(angle)+uni(rng), 0.1f*height+heightSpacing*(i/numPerRing), averageRad*std::sin(angle)+uni(rng))));
        physx::PxRigidActorExt::createExclusiveShape(*ballActor, physx::PxSphereGeometry(ballRad), *mBallMaterial);
        float density = 1;
        physx::PxRigidBodyExt::updateMassAndInertia(*ballActor, &density, 1, nullptr, false);
        mScene->addActor(*ballActor);
        physx::PxRigidBodyExt::addLocalForceAtLocalPos(*ballActor, physx::PxVec3(uni(rng), 0, uni(rng)), physx::PxVec3(0,0,0));
        ballActors.push_back(ballActor);
    }
    
    // run simulation
    for (int i = 0; i < 500; ++i) {
        mScene->simulate(1.0f / 1000.0f);
        mScene->fetchResults(true);
    }
    std::vector<physx::PxVec3> ballPositions;
    ballPositions.reserve(numBalls);
    for (physx::PxRigidDynamic* ball : ballActors) {
        ballPositions.push_back(ball->getGlobalPose().p);
    }
    std::filesystem::path path{ "D://pppl2023/ballsimulation/ballpositions/6mm.csv" }; //creates TestingFolder object on C:
    std::filesystem::create_directories(path.parent_path()); //add directories based on the object path (without this line it will not work)
    std::ofstream ofs(path);
    ofs << "x,y,z\n";
    for (const physx::PxVec3& position : ballPositions) {
        std::array<char, 50> str;
        auto to_chars_result = std::to_chars(str.data(), str.data() + str.size(), position.x, std::chars_format::fixed);
        if (to_chars_result.ec != std::errc()) {
            throw("failed to char convert");
        }
        ofs << std::string_view(str.data(), to_chars_result.ptr-str.data());
        ofs << ",";

        to_chars_result = std::to_chars(str.data(), str.data() + str.size(), position.y, std::chars_format::fixed);
        if (to_chars_result.ec != std::errc()) {
            throw("failed to char convert");
        }
        ofs << std::string_view(str.data(), to_chars_result.ptr - str.data());
        ofs << ",";

        to_chars_result = std::to_chars(str.data(), str.data() + str.size(), position.z, std::chars_format::fixed);
        if (to_chars_result.ec != std::errc()) {
            throw("failed to char convert");
        }
        ofs << std::string_view(str.data(), to_chars_result.ptr - str.data());
        ofs << "\n";

    }
    ofs.close();
}