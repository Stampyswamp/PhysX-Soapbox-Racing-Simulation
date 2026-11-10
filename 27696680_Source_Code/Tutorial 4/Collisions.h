#pragma once
#include "PxPhysicsAPI.h"
using namespace physx;

enum CollisionGroup
{
    CG_CAR = 1 << 0,
    CG_TRACK = 1 << 1,
    CG_WORLD = 1 << 2,
    CG_TRIGGER = 1 << 3
};

static inline void ApplyMaterialAndGroup(
    PxShape* shape,
    PxMaterial* mat,
    PxU32 group,
    PxU32 mask)
{
    shape->setMaterials(&mat, 1);

    PxFilterData fd;
    fd.word0 = group;
    fd.word1 = mask;
    fd.word2 = 0;
    fd.word3 = 0;

    shape->setSimulationFilterData(fd);
}
