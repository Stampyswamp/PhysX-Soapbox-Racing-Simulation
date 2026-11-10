#pragma once

#include "PxPhysicsAPI.h"
using namespace physx;

namespace PhysicsEngine
{
    struct RampConfig
    {
        float width = 4.0f;
        float length = 20.0f;
        float height = 0.0f;
        int   segments = 40;
        float noiseAmp = 0.0f;
        int   interiorCount = 5;

        float railOffset = 0.1f;
        float railHeight = 5.0f;
    };

    struct TurnConfig
    {
        float radius = 30.0f;
        float width = 4.0f;
        float height = 0.0f;
        int   samples = 32;
    };

    struct RailConfig
    {
        float offsets[2] = { -2.0f, 2.0f };
        float heightOffset = 0.8f;
        float thickness = 0.2f;
        PxVec3 color = PxVec3(0.2f, 0.2f, 0.2f);
    };

    struct HayBaleConfig
    {
        float baleSize = 1.0f;
        int cols = 16;
        int rows = 6;
    };

    struct MaterialSet
    {
        PxMaterial* trackMat = nullptr;
        PxMaterial* railMat = nullptr;
        PxMaterial* hayMat = nullptr;
        PxMaterial* carMat = nullptr;
        PxMaterial* worldMat = nullptr;
        PxMaterial* defaultMat = nullptr;
    };

    inline MaterialSet CreateMaterials(PxPhysics* physics)
    {
        MaterialSet m;

        m.carMat = physics->createMaterial(0.05f, 0.1f, 0.15f);
        m.trackMat = physics->createMaterial(0.7f, 0.08f, 0.05f);
        m.worldMat = physics->createMaterial(0.5f, 0.5f, 0.1f);
        m.railMat = physics->createMaterial(0.6f, 0.0f, 0.3f);
        m.hayMat = physics->createMaterial(0.2f, 0.1f, 0.4f);
        m.defaultMat = physics->createMaterial(0.3f, 0.3f, 0.0f);

        return m;
    }
}
